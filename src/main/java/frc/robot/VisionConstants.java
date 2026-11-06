package frc.robot;

import java.util.List;
import java.util.stream.IntStream;

import frc.lib.vision.CameraConfig;

/**
 * This season's vision setup. Each year, change this file; frc.lib.vision stays the same.
 */
public final class VisionConstants {
    private VisionConstants() {}

    /*
     * Every Limelight on the robot. To add a camera:
     *   1. In its web UI, give it a unique hostname and static IP (e.g. limelight-back, 10.83.52.12).
     *   2. Measure where it's mounted (see CameraConfig for directions and units).
     *   3. Add a line here.
     * TODO: MEASURE the mount values on the real robot. Wrong values make every vision pose wrong,
     * and with two cameras they make the cameras disagree.
     */
    public static final List<CameraConfig> kCameras = List.of(
        //               name          fwd  side  up   roll pitch yaw  trust
        new CameraConfig("limelight",  0.0, 0.0,  0.0, 0.0, 0.0,  0.0, 1.0)
        // new CameraConfig("limelight-back", 0.0, 0.0, 0.0, 0.0, 0.0, 180.0, 1.0)
    );

    /*
     * Every tag ID we may use: REBUILT field tags 1-32 (check against the field layout).
     * Leaving ALIGNMENT mode restores the camera filter to this list, so add any shop tag IDs
     * that aren't in it, or the cameras will ignore them.
     */
    public static final int[] kAllTagIds = IntStream.rangeClosed(1, 32).toArray();
}
