// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import org.wpilib.command3.Scheduler;
import org.wpilib.math.linalg.VecBuilder;

/**
 * One Limelight camera. It spots AprilTags and uses them to fix up the robot's idea of where it is
 * on the field. Call {@link #registerAll} once from {@link frc.robot.Robot} to set up every camera.
 *
 * <p>This is <b>not</b> a {@code Mechanism}. It moves nothing - it only reads the camera and
 * corrects odometry. {@link #registerAll} puts {@link #update()} on the scheduler so it runs every
 * loop.
 *
 * <p>With two or more tags in view it uses MegaTag1, which measures heading from the tags. With one
 * tag it uses MegaTag2, which trusts the gyro's heading instead - so seed the gyro, or single-tag
 * results will be off. Does nothing in simulation (there is no camera).
 */
public class Limelight {
  // How much to trust a sighting: stdDev = coefficient * distance^1.2 / tagCount^2.
  // Bigger stdDev = trust it less. Far tags count less; more tags count more.
  private static final double XY_STD_DEV_COEFFICIENT = 0.333;
  private static final double ROTATION_STD_DEV_COEFFICIENT = 1.5; // MegaTag1 heading trust
  private static final double MAX_TAG_DISTANCE_METERS = 4.0; // skip far, noisy tags
  private static final double IGNORE_VISION_HEADING = 9_999_999; // huge = let the gyro own heading

  private final String name;
  private final DriveMechanism drivetrain;

  private Limelight(String name, DriveMechanism drivetrain) {
    this.name = name;
    this.drivetrain = drivetrain;
  }

  /**
   * Creates one camera for each name and puts them all on the scheduler, so every camera updates
   * every loop. The names must match each camera's name in NetworkTables.
   */
  public static void registerAll(DriveMechanism drivetrain, String... cameraNames) {
    for (String name : cameraNames) {
      Limelight camera = new Limelight(name, drivetrain);
      Scheduler.getDefault().addPeriodic(camera::update);
    }
    // One flush per loop sends every camera's heading write in a single batch.
    Scheduler.getDefault().addPeriodic(LimelightHelpers::Flush);
  }

  /** Runs one vision update for this camera. */
  private void update() {
    // Tell the camera which way we are facing (MegaTag2 needs it). NoFlush means the shared
    // flush in registerAll sends it.
    double headingDegrees = drivetrain.getPose().getRotation().getDegrees();
    LimelightHelpers.SetRobotOrientation_NoFlush(name, headingDegrees, 0, 0, 0, 0, 0);

    // MegaTag1 for 2+ tags; switch to MegaTag2 for a single tag.
    PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
    if (LimelightHelpers.validPoseEstimate(estimate) && estimate.tagCount == 1) {
      estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
    }

    // Skip if no tag is in view, or the tags are too far to trust.
    if (!LimelightHelpers.validPoseEstimate(estimate)
        || estimate.avgTagDist > MAX_TAG_DISTANCE_METERS) {
      return;
    }

    // Closer tags and more tags earn more trust. MegaTag1 gives a real heading; MegaTag2 leaves
    // the heading to the gyro.
    double distanceFactor = Math.pow(estimate.avgTagDist, 1.2);
    double tagFactor = estimate.tagCount * estimate.tagCount;
    double xyStdDev = XY_STD_DEV_COEFFICIENT * distanceFactor / tagFactor;
    double headingStdDev =
        estimate.isMegaTag2
            ? IGNORE_VISION_HEADING
            : ROTATION_STD_DEV_COEFFICIENT * distanceFactor / tagFactor;
    drivetrain.addVisionMeasurement(
        estimate.pose,
        estimate.timestampSeconds,
        VecBuilder.fill(xyStdDev, xyStdDev, headingStdDev));
  }
}
