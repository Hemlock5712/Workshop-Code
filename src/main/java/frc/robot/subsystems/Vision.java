// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.limelightvision.Limelight;
import com.limelightvision.LimelightResults;
import com.limelightvision.PoseEstimate;
import com.limelightvision.PoseEstimateType;
import java.util.ArrayList;
import java.util.List;
import org.wpilib.command3.Scheduler;
import org.wpilib.math.linalg.VecBuilder;

/**
 * Feeds AprilTag sightings from every Limelight on the robot into the drivetrain's pose estimator,
 * so the camera pulls odometry back toward the truth.
 *
 * <p>A camera drives nothing, so this is not a {@code Mechanism}. It runs from the scheduler's
 * periodic list instead, every loop and in every mode.
 */
public class Vision {
  // Trust weighting. A sighting's error bar grows with distance and shrinks with tag count.
  private static final double XY_STD_DEV_COEFFICIENT = 0.333;
  private static final double ROTATION_STD_DEV_COEFFICIENT = 1.5;
  private static final double MAX_TAG_DISTANCE_METERS = 4.0;
  // MegaTag2 solved from our own heading, so its heading is not a new measurement.
  private static final double IGNORE_VISION_HEADING = 9_999_999;
  private static final int MIN_TAGS_FOR_MEGATAG1 = 2;

  private final DriveMechanism drivetrain;
  private final List<Limelight> cameras = new ArrayList<>();

  /**
   * @param drivetrain the drivetrain whose pose estimate the cameras correct
   * @param cameraNames each camera's NetworkTables name, exactly as set in its web interface
   */
  public Vision(DriveMechanism drivetrain, String... cameraNames) {
    this.drivetrain = drivetrain;
    for (String name : cameraNames) {
      Limelight camera = new Limelight(name);
      // Read the heading that update() shares, for MegaTag2.
      camera.setUseSharedOrientation(true);
      cameras.add(camera);
    }
    Scheduler.getDefault().addPeriodic(() -> update());
  }

  private void update() {
    // One write reaches every camera that reads the shared heading.
    Limelight.setSharedRobotOrientation(drivetrain.getPose().getRotation().getDegrees());

    for (Limelight camera : cameras) {
      // Every frame since the last loop, so none is skipped and none is used twice.
      for (LimelightResults frame : camera.readResultsQueue()) {
        addFrame(camera, frame);
      }
    }
  }

  private void addFrame(Limelight camera, LimelightResults frame) {
    PoseEstimate estimate = camera.getPoseEstimate(frame, PoseEstimateType.MT1_WPIBLUE);
    if (estimate.fieldedTagCount < MIN_TAGS_FOR_MEGATAG1) {
      estimate = camera.getPoseEstimate(frame, PoseEstimateType.MT2_WPIBLUE);
    }

    // Zero means the library's own gates found nothing wrong with the frame.
    if (estimate.rejectionFlags != 0 || estimate.avgTagDistanceMeters > MAX_TAG_DISTANCE_METERS) {
      return;
    }

    double distanceFactor = Math.pow(estimate.avgTagDistanceMeters, 1.2);
    double tagFactor = estimate.fieldedTagCount * estimate.fieldedTagCount;
    double xyStdDev = XY_STD_DEV_COEFFICIENT * distanceFactor / tagFactor;
    double headingStdDev =
        estimate.isMT2()
            ? IGNORE_VISION_HEADING
            : ROTATION_STD_DEV_COEFFICIENT * distanceFactor / tagFactor;

    // The picture's own timestamp: the estimator winds back to when it was taken.
    drivetrain.addVisionMeasurement(
        estimate.pose,
        estimate.timestampSeconds,
        VecBuilder.fill(xyStdDev, xyStdDev, headingStdDev));
  }
}
