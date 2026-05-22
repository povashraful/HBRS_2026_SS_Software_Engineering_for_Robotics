import math

from State_Machine.config import SAFE_DISTANCE


def is_collision_detected(
    ranges,
    safe_distance=SAFE_DISTANCE
):
    valid_ranges = []

    for r in ranges:

        if (
            not math.isnan(r)
            and not math.isinf(r)
            and r > 0.0
        ):
            valid_ranges.append(r)

    if len(valid_ranges) == 0:
        return False

    return min(valid_ranges) < safe_distance
