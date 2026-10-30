"""Find the SentinelAI ESP32 on the local network (robot.host: auto).

A phone hotspot or a router gives the ESP32 a new IP address now and then, so typing the
address into settings.yaml breaks easily. With ``host: auto`` the laptop looks for the robot
itself, in this order:

1. the address that worked last time (``config/robot_last_host.txt``, written automatically);
2. the name ``sentinel-robot.local`` (mDNS - the firmware announces it; works on most
   Windows 10/11 laptops, not on every network);
3. a scan of the laptop's own local network(s) (``x.y.z.1`` - ``x.y.z.254``) on port 3333.

A device only counts as the robot when it answers with the SentinelAI ``hello`` message
(protocol 3). Only private (LAN) addresses are scanned - never the internet. Each probe opens
a connection and closes it again; the ESP32 prints a short "connected / disconnected" for it,
nothing else happens (a disconnect never starts or resumes anything).
"""
from __future__ import annotations

import ipaddress
import json
import logging
import socket
import threading
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

log = logging.getLogger(__name__)

AUTO_NAMES = ("", "auto")
MDNS_NAME = "sentinel-robot.local"
CACHE_FILE = Path(__file__).resolve().parent.parent / "config" / "robot_last_host.txt"


def is_auto(host: str | None) -> bool:
    return (host or "").strip().lower() in AUTO_NAMES


def probe(host: str, port: int = 3333, connect_timeout_s: float = 0.4, hello_timeout_s: float = 1.2) -> dict | None:
    """Connect, wait for the robot's hello line, close. Returns the hello dict or None."""
    try:
        sock = socket.create_connection((host, port), timeout=connect_timeout_s)
    except OSError:
        return None
    try:
        sock.settimeout(hello_timeout_s)
        buf = b""
        deadline = time.monotonic() + hello_timeout_s
        while b"\n" not in buf and len(buf) < 1024 and time.monotonic() < deadline:
            chunk = sock.recv(512)
            if not chunk:
                break
            buf += chunk
        line = buf.split(b"\n", 1)[0]
        msg = json.loads(line.decode("ascii", "replace"))
        if isinstance(msg, dict) and msg.get("type") == "event" and msg.get("event") == "hello" \
                and msg.get("protocol") == 3:
            return msg
    except (OSError, ValueError):
        pass
    finally:
        try:
            sock.close()
        except OSError:
            pass
    return None


def _resolve(name: str, timeout_s: float = 2.0) -> str | None:
    """getaddrinfo with a time limit (an unanswered mDNS lookup can block for many seconds)."""
    out: list[str] = []

    def run() -> None:
        try:
            for fam, _, _, _, addr in socket.getaddrinfo(name, None, socket.AF_INET, socket.SOCK_STREAM):
                out.append(addr[0])
        except OSError:
            pass

    t = threading.Thread(target=run, daemon=True)
    t.start()
    t.join(timeout_s)
    return out[0] if out else None


def local_networks() -> list[ipaddress.IPv4Network]:
    """The laptop's private IPv4 networks, as /24 blocks (largest a hotspot/home router uses in practice)."""
    addrs: set[str] = set()
    try:   # the address used for the default route; no packet is sent by a UDP connect
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect(("10.255.255.255", 1))
            addrs.add(s.getsockname()[0])
        finally:
            s.close()
    except OSError:
        pass
    try:
        addrs.update(socket.gethostbyname_ex(socket.gethostname())[2])
    except OSError:
        pass
    nets: list[ipaddress.IPv4Network] = []
    for a in addrs:
        try:
            ip = ipaddress.IPv4Address(a)
        except ValueError:
            continue
        if not ip.is_private or ip.is_loopback or ip.is_link_local:
            continue
        net = ipaddress.IPv4Network(f"{a}/24", strict=False)
        if net not in nets:
            nets.append(net)
    return nets


def read_cache() -> str | None:
    try:
        text = CACHE_FILE.read_text(encoding="utf-8").strip()
        ipaddress.IPv4Address(text)
        return text
    except (OSError, ValueError):
        return None


def write_cache(host: str) -> None:
    try:
        ipaddress.IPv4Address(host)
        CACHE_FILE.write_text(host + "\n", encoding="utf-8")
    except (OSError, ValueError):
        pass


def find_robot(port: int = 3333, stop: threading.Event | None = None, workers: int = 64,
               use_mdns: bool = True, use_scan: bool = True, extra_hosts: list[str] | None = None) -> str | None:
    """Return the robot's IP address, or None if it is not on any local network right now."""
    tried: set[str] = set()

    def check(h: str | None) -> str | None:
        if not h or h in tried or (stop is not None and stop.is_set()):
            return None
        tried.add(h)
        return h if probe(h, port) else None

    for h in [read_cache(), *(extra_hosts or [])]:
        if check(h):
            log.info("Robot found at %s (last known address)", h)
            return h
    if use_mdns:
        h = _resolve(MDNS_NAME)
        if check(h):
            log.info("Robot found at %s (%s)", h, MDNS_NAME)
            write_cache(h)
            return h
    if not use_scan:
        return None
    nets = local_networks()
    if not nets:
        log.warning("Robot search: the laptop has no local network address (Wi-Fi off?)")
        return None
    candidates = [str(ip) for net in nets for ip in net.hosts() if str(ip) not in tried]
    log.info("Searching for the robot on %s (port %d) ...", ", ".join(str(n) for n in nets), port)
    found: str | None = None
    with ThreadPoolExecutor(max_workers=workers) as pool:
        futures = {pool.submit(probe, h, port): h for h in candidates}
        for fut in as_completed(futures):
            if fut.result():
                found = futures[fut]
                break
            if stop is not None and stop.is_set():
                break
        for f in futures:
            f.cancel()
    if found:
        log.info("Robot found at %s (network scan)", found)
        write_cache(found)
    return found
