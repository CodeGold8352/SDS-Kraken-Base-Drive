package frc.lib.vision;

/**
 * One Limelight on the robot: its name and where it is mounted.
 *
 * Mount position is measured from the center of the robot at floor level.
 * Forward/side/up are in meters (side is + to the LEFT).
 * Roll/pitch/yaw are in degrees (pitch is + tilted UP, yaw is + turned LEFT).
 *
 * @param name           Hostname set in the Limelight web UI, e.g. "limelight-front". Every camera needs its own.
 * @param forwardMeters  How far in front of robot center
 * @param sideMeters     How far left of robot center
 * @param upMeters       How high above the floor
 * @param rollDegrees    Tilt side to side
 * @param pitchDegrees   Tilt up/down
 * @param yawDegrees     Which way it points (0 = robot front, 180 = robot back)
 * @param stdDevScale    Extra trust multiplier for this camera. 1.0 = normal; bigger = trust it less
 *                       (e.g. a low-resolution or wobbly mount).
 */
public record CameraConfig(
    String name,
    double forwardMeters,
    double sideMeters,
    double upMeters,
    double rollDegrees,
    double pitchDegrees,
    double yawDegrees,
    double stdDevScale
) {}
