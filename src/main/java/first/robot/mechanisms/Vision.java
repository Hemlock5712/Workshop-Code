// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.mechanisms;

import com.limelightvision.Limelight;
import com.limelightvision.LimelightResults;
import com.limelightvision.PoseEstimate;
import com.limelightvision.PoseEstimateType;
import java.util.ArrayList;
import java.util.List;
import org.wpilib.command3.Scheduler;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.telemetry.TelemetryTable;

/**
 * Feeds AprilTag sightings from every Limelight on the robot into the drivetrain's pose estimator,
 * so the camera pulls odometry back toward the truth.
 *
 * <p>A camera drives nothing, so this is not a {@code Mechanism}. It runs from the scheduler's
 * periodic list instead, every loop and in every mode.
 */
public class Vision {
  private final CommandSwerveDrivetrain drivetrain;
  private final List<Limelight> cameras = new ArrayList<>();

  /**
   * @param drivetrain the drivetrain whose pose estimate the cameras correct
   * @param cameraNames each camera's NetworkTables name, exactly as set in its web interface
   */
  public Vision(CommandSwerveDrivetrain drivetrain, String... cameraNames) {
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
    // MegaTag1 solves position and heading from the tags alone. It needs two tags to do it well.
    PoseEstimate estimate = camera.getPoseEstimate(frame, PoseEstimateType.MT1_WPIBLUE);
    if (estimate.fieldedTagCount < 2) {
      // With one tag, ask again as MegaTag2. It takes our heading as given and solves only
      // position.
      estimate = camera.getPoseEstimate(frame, PoseEstimateType.MT2_WPIBLUE);
    }

    // Zero flags means the library found nothing wrong with the frame. Past 4 meters a tag is too
    // small in the picture to trust.
    boolean accepted = estimate.rejectionFlags == 0 && estimate.avgTagDistanceMeters <= 4.0;

    // Watch these in AdvantageScope. Pose is where the camera says the robot is.
    TelemetryTable table = Telemetry.getTable("Vision");
    table.log("Accepted", accepted);
    table.log("TagCount", estimate.fieldedTagCount);
    if (!accepted) {
      return;
    }
    table.log("Pose", estimate.pose);

    // A standard deviation says how far off a sighting might be, and it sets how far each picture
    // moves the pose toward where the camera says the robot is. That share is 0.1 / (0.1 + stdDev):
    // 0.1 moves it halfway, 0.5 a sixth of the way, 0.9 a tenth. Bigger means calmer.
    // Start at 0.5 for one tag one meter away. Trust a far tag less and several tags more.
    double xyStdDev = 0.5 * estimate.avgTagDistanceMeters / estimate.fieldedTagCount; // meters
    double headingStdDev =
        1.0 * estimate.avgTagDistanceMeters / estimate.fieldedTagCount; // radians
    if (estimate.isMT2()) {
      // MegaTag2 solved from our own heading, so its heading is not a new measurement. A huge
      // number tells the estimator to ignore it.
      headingStdDev = 9_999_999;
    }

    // The picture's own timestamp: the estimator winds back to when it was taken.
    drivetrain.addVisionMeasurement(
        estimate.pose,
        estimate.timestampSeconds,
        VecBuilder.fill(xyStdDev, xyStdDev, headingStdDev));
  }
}
