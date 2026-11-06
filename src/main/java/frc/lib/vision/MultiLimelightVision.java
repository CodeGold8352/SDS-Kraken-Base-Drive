package frc.lib.vision;

import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.LimelightHelpers;

/**
 * Corrects the drivetrain's position estimate using any number of Limelights (MegaTag2).
 *
 * Each camera sends its own estimate to the drivetrain's pose estimator, which blends them
 * with wheel odometry. We never average cameras ourselves: the estimator already weighs each
 * one by how much we trust it (its standard deviation).
 *
 * This class does not know about the game or the drivetrain, so it can be reused every season.
 * What changes each year (camera list, tag IDs) lives in the robot's VisionConstants.
 *
 * Every loop, for each camera we:
 *   1. Tell it which way the robot is facing (MegaTag2 needs the gyro heading, every loop).
 *   2. Ask it where it thinks the robot is.
 *   3. If that answer looks trustworthy, send it to the pose estimator.
 */
public class MultiLimelightVision extends SubsystemBase {
    /** Where accepted vision measurements go. Timestamps are FPGA seconds (the Limelight's clock). */
    @FunctionalInterface
    public interface VisionMeasurementConsumer {
        void accept(Pose2d pose, double timestampSeconds, Matrix<N3, N1> stdDevs);
    }

    /* Ignore vision while spinning faster than this; the image blurs and the heading lags */
    private static final double kMaxYawRateDegPerSec = 720;

    /* Ignore estimates when the tags are farther than this on average; far tags are noisy */
    private static final double kMaxAvgTagDistMeters = 4.0;

    /*
     * How much to trust vision (standard deviation for x and y, in meters). Bigger = trust less.
     *   stdDev = kStdDevPerMeterSquared * distance² / tagCount * camera.stdDevScale
     * Far tags are trusted much less (distance squared); seeing more tags is trusted more.
     * Example: one tag 2 m away -> 0.1 * 4 / 1 = 0.4 m.
     */
    private static final double kStdDevPerMeterSquared = 0.1;

    /* Heading is set huge so vision never changes it; the gyro is better at that (and MegaTag2 uses it) */
    private static final double kHeadingStdDev = 9999999;

    /* ALIGNMENT only: within this distance of the target tags, trust vision even more */
    private static final double kAlignmentCloseRangeMeters = 1.5;
    private static final double kAlignmentCloseRangeScale = 0.5;

    private final List<CameraConfig> m_cameras;
    private final int[] m_allTagIds;
    private final Supplier<Pose2d> m_robotPose;
    private final DoubleSupplier m_yawRateRadPerSec;
    private final VisionMeasurementConsumer m_measurementConsumer;

    /* Shows the robot pose plus each camera's own estimate, so you can see if cameras agree */
    private final Field2d m_field = new Field2d();

    /* When false, vision is ignored and the robot relies on wheel odometry only */
    private boolean m_enabled = true;
    private VisionProfile m_profile = VisionProfile.LOCALIZATION;

    /**
     * @param cameras             Every Limelight on the robot
     * @param allTagIds           Every tag ID we may use (the field's tags, plus any shop tags).
     *                            Used to undo the ALIGNMENT tag filter.
     * @param robotPose           Current estimated robot pose
     * @param yawRateRadPerSec    How fast the robot is turning, counterclockwise-positive
     * @param measurementConsumer Usually the drivetrain's addVisionMeasurement
     */
    public MultiLimelightVision(
        List<CameraConfig> cameras,
        int[] allTagIds,
        Supplier<Pose2d> robotPose,
        DoubleSupplier yawRateRadPerSec,
        VisionMeasurementConsumer measurementConsumer
    ) {
        m_cameras = cameras;
        m_allTagIds = allTagIds;
        m_robotPose = robotPose;
        m_yawRateRadPerSec = yawRateRadPerSec;
        m_measurementConsumer = measurementConsumer;

        // Tell each Limelight where it is mounted, so it can work out where the robot is
        for (CameraConfig cam : m_cameras) {
            LimelightHelpers.setCameraPose_RobotSpace(cam.name(),
                cam.forwardMeters(), cam.sideMeters(), cam.upMeters(),
                cam.rollDegrees(), cam.pitchDegrees(), cam.yawDegrees());
        }

        SmartDashboard.putData("Vision Field", m_field);
    }

    public void setEnabled(boolean enabled) {
        m_enabled = enabled;
    }

    /** Command that turns vision corrections on or off (used as a PathPlanner named command). */
    public Command setEnabledCommand(boolean enabled) {
        return Commands.runOnce(() -> setEnabled(enabled));
    }

    /** Normal mode: use every tag. Also undoes any ALIGNMENT tag filter. */
    public void useLocalization() {
        m_profile = VisionProfile.LOCALIZATION;
        setTagFilter(m_allTagIds);
    }

    /** Line-up mode: only use these tags (e.g. the ones on the scoring spot we're driving to). */
    public void useAlignment(int... targetTagIds) {
        m_profile = VisionProfile.ALIGNMENT;
        setTagFilter(targetTagIds);
    }

    /**
     * Command that uses ALIGNMENT while it runs, then always goes back to LOCALIZATION when it ends
     * or is interrupted. Use this instead of useAlignment() so the tag filter can't get stuck on.
     * Example: joystick.rightBumper().whileTrue(driveToScore.alongWith(vision.alignmentCommand(7)));
     */
    public Command alignmentCommand(int... targetTagIds) {
        return Commands.startEnd(() -> useAlignment(targetTagIds), this::useLocalization);
    }

    public VisionProfile getProfile() {
        return m_profile;
    }

    @Override
    public void periodic() {
        Pose2d robotPose = m_robotPose.get();
        double yawRateDegPerSec = Units.radiansToDegrees(m_yawRateRadPerSec.getAsDouble());
        boolean spinningTooFast = Math.abs(yawRateDegPerSec) > kMaxYawRateDegPerSec;

        m_field.setRobotPose(robotPose);

        for (CameraConfig cam : m_cameras) {
            // 1. MegaTag2 needs the robot's heading every loop, even when we're not using the result
            LimelightHelpers.SetRobotOrientation(cam.name(),
                robotPose.getRotation().getDegrees(), 0, 0, 0, 0, 0);

            // 2. Where does this camera think we are?
            var estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cam.name());
            boolean seesTags = estimate != null && estimate.tagCount > 0;
            boolean tooFar = seesTags && estimate.avgTagDist > kMaxAvgTagDistMeters;
            boolean accepted = m_enabled && seesTags && !spinningTooFast && !tooFar;

            // 3. Send it to the pose estimator only if it looks trustworthy
            if (accepted) {
                double xyStdDev = xyStdDev(cam, estimate.avgTagDist, estimate.tagCount);
                m_measurementConsumer.accept(
                    estimate.pose,
                    estimate.timestampSeconds,
                    VecBuilder.fill(xyStdDev, xyStdDev, kHeadingStdDev)
                );
            }

            String prefix = "Vision/" + cam.name() + "/";
            SmartDashboard.putNumber(prefix + "Tag Count", seesTags ? estimate.tagCount : 0);
            SmartDashboard.putBoolean(prefix + "Accepted", accepted);
            if (seesTags) {
                m_field.getObject(cam.name()).setPose(estimate.pose);
            } else {
                m_field.getObject(cam.name()).setPoses(); // hide it when this camera sees nothing
            }
        }

        SmartDashboard.putBoolean("Vision/Enabled", m_enabled);
        SmartDashboard.putString("Vision/Profile", m_profile.name());
    }

    /** How much to trust one estimate, in meters. See kStdDevPerMeterSquared for the formula. */
    private double xyStdDev(CameraConfig cam, double avgTagDistMeters, int tagCount) {
        double stdDev = kStdDevPerMeterSquared * avgTagDistMeters * avgTagDistMeters / tagCount
            * cam.stdDevScale();
        if (m_profile == VisionProfile.ALIGNMENT && avgTagDistMeters < kAlignmentCloseRangeMeters) {
            stdDev *= kAlignmentCloseRangeScale;
        }
        return stdDev;
    }

    private void setTagFilter(int[] tagIds) {
        for (CameraConfig cam : m_cameras) {
            LimelightHelpers.SetFiducialIDFiltersOverride(cam.name(), tagIds);
        }
    }
}
