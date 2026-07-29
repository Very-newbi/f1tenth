"""Small helpers for converting vehicle commands to VESC commands."""

ERPM_GAIN = 4614.0
MIN_DRIVE_ERPM = 1850.0
MIN_DRIVE_SPEED_MS = MIN_DRIVE_ERPM / ERPM_GAIN


def clamp(value, lower, upper):
    """Clamp a numeric value to an inclusive range."""
    return max(lower, min(value, upper))


def speed_to_erpm(speed_mps, erpm_gain=ERPM_GAIN):
    """Convert speed in m/s to VESC ERPM."""
    return speed_mps * erpm_gain


def apply_min_drive_speed(
    speed_mps,
    deadband=0.02,
    min_drive_speed=MIN_DRIVE_SPEED_MS,
):
    """Apply minimum rolling speed so small positive commands can move the car."""
    if abs(speed_mps) < deadband:
        return 0.0
    if speed_mps > 0.0:
        return max(speed_mps, min_drive_speed)
    return min(speed_mps, -min_drive_speed)
