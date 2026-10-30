"""Plain-language fall assessment shown on the dashboard for every person.

The labels deliberately separate what the model found from what it could not see:

  fall confirmed by model     the fall state machine reached FALL_CONFIRMED
  possible fall               POSSIBLE_FALL or FALL_SUSPECTED (not yet confirmed)
  recovering                  RECOVERY_OR_RESET (person got up / operator reset)
  unknown - poor visibility   too few body keypoints visible to judge
  lying, no fall evidence     posture LYING but no fall was seen (e.g. lying on a bed/sofa)
  no fall evidence            nothing suspicious seen
  tracking lost               a person in a fall state left the picture / was lost by the tracker
"""
from __future__ import annotations

NO_EVIDENCE = "no fall evidence"
POSSIBLE = "possible fall"
CONFIRMED = "fall confirmed by model"
LYING_NO_EVIDENCE = "lying, no fall evidence"
UNKNOWN_VISIBILITY = "unknown - poor visibility"
TRACKING_LOST = "tracking lost"
RECOVERING = "recovering"
LABELS = (NO_EVIDENCE, POSSIBLE, CONFIRMED, LYING_NO_EVIDENCE, UNKNOWN_VISIBILITY, TRACKING_LOST, RECOVERING)


def assess(fall_state: str, posture: str, reliability: float, min_reliability: float = 0.25) -> str:
    if fall_state == "FALL_CONFIRMED":
        return CONFIRMED
    if fall_state in ("POSSIBLE_FALL", "FALL_SUSPECTED"):
        return POSSIBLE
    if fall_state == "RECOVERY_OR_RESET":
        return RECOVERING
    if reliability < min_reliability or posture == "UNKNOWN":
        return UNKNOWN_VISIBILITY
    if posture == "LYING":
        return LYING_NO_EVIDENCE
    return NO_EVIDENCE
