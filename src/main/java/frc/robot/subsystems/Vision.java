package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.LimelightHelpers;

/**
 * Uses a Limelight 4 to correct the drivetrain's position estimate with AprilTags (MegaTag2).
 *
 * Every loop we:
 *   1. Tell the Limelight which way the robot is facing (MegaTag2 needs the gyro heading).
 *   2. Ask it where it thinks the robot is on the field.
 *   3. If that answer looks trustworthy, blend it into the drivetrain's pose estimator.
 *
 * The Limelight finds tags using the field map loaded in its web UI (http://limelight.local:5801).
 * In the shop, that must be a custom map matching where the tags are actually taped up.
 */
public class Vision extends SubsystemBase {
    /* Must match the Limelight's name in its web UI (default is "limelight") */
    private static final String kLimelightName = "limelight";

    /*
     * Where the camera sits on the robot, measured from the center of the robot at floor level.
     * Forward/side/up in meters (side is + to the LEFT), roll/pitch/yaw in degrees (pitch is + tilted UP).
     * TODO: MEASURE THESE on the real robot. Wrong values here make every vision pose wrong.
     */
    private static final double kCameraForward = 0.0;
    private static final double kCameraSide = 0.0;
    private static final double kCameraUp = 0.0;
    private static final double kCameraRoll = 0.0;
    private static final double kCameraPitch = 0.0;
    private static final double kCameraYaw = 0.0;

    /* Ignore vision while spinning faster than this; the camera image blurs and the heading lags */
    private static final double kMaxYawRateDegPerSec = 720;

    /*
     * How much to trust vision vs. wheel odometry (standard deviations: x meters, y meters, heading radians).
     * Bigger = trust less. 0.7 m is Limelight's recommended starting point for MegaTag2.
     * Heading is set huge so vision never changes the heading; the Pigeon gyro is better at that.
     */
    private static final double kXYStdDev = 0.7;
    private static final double kHeadingStdDev = 9999999;

    private final CommandSwerveDrivetrain m_drivetrain;

    /* When false, vision is ignored and the robot relies on wheel odometry only */
    private boolean m_enabled = true;

    public Vision(CommandSwerveDrivetrain drivetrain) {
        m_drivetrain = drivetrain;
        LimelightHelpers.setCameraPose_RobotSpace(kLimelightName,
            kCameraForward, kCameraSide, kCameraUp, kCameraRoll, kCameraPitch, kCameraYaw);
    }

    public void setEnabled(boolean enabled) {
        m_enabled = enabled;
    }

    /** Command that turns vision corrections on or off (used as a PathPlanner named command). */
    public Command setEnabledCommand(boolean enabled) {
        return Commands.runOnce(() -> setEnabled(enabled));
    }

    @Override
    public void periodic() {
        var state = m_drivetrain.getState();
        double yawRateDegPerSec = Units.radiansToDegrees(state.Speeds.omegaRadiansPerSecond);

        // 1. MegaTag2 needs the robot's heading every loop, even when we're not using the result
        LimelightHelpers.SetRobotOrientation(kLimelightName,
            state.Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);

        // 2. Where does the Limelight think we are?
        var estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(kLimelightName);
        boolean seesTags = estimate != null && estimate.tagCount > 0;
        boolean spinningTooFast = Math.abs(yawRateDegPerSec) > kMaxYawRateDegPerSec;
        boolean accepted = m_enabled && seesTags && !spinningTooFast;

        // 3. Blend it in only if it looks trustworthy
        if (accepted) {
            m_drivetrain.addVisionMeasurement(
                estimate.pose,
                estimate.timestampSeconds,
                VecBuilder.fill(kXYStdDev, kXYStdDev, kHeadingStdDev)
            );
        }

        SmartDashboard.putBoolean("Vision/Enabled", m_enabled);
        SmartDashboard.putNumber("Vision/Tag Count", seesTags ? estimate.tagCount : 0);
        SmartDashboard.putBoolean("Vision/Accepted", accepted);
    }
}
