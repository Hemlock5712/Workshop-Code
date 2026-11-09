// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.limelightvision.FiducialTarget;
import com.limelightvision.Limelight;
import com.limelightvision.LimelightResults;
import com.limelightvision.PoseEstimate;
import com.limelightvision.PoseEstimateType;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.wpilib.command3.Scheduler;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.telemetry.Telemetry;

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

  // Where each tag sat in the drivetrain's pose frame the last time a camera saw it, by tag ID.
  private final Map<Integer, Pose2d> tagPoses = new HashMap<>();

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
    // Before any gate: aligning to one close tag is exactly the sighting the gates distrust.
    rememberTags(frame, estimate.timestampSeconds);

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

  /**
   * Records where every tag in the frame sits, in the same frame as {@code drivetrain.getPose()}.
   * The robot's pose at the moment the picture was taken, plus the tag's offset from the robot,
   * puts the tag down once. Between pictures, the wheels carry the robot and the tag stays put.
   */
  private void rememberTags(LimelightResults frame, double timestampSeconds) {
    Optional<Pose2d> poseThen = drivetrain.samplePoseAt(timestampSeconds);
    if (poseThen.isEmpty()) {
      return;
    }
    for (FiducialTarget tag : frame.fiducialTargets) {
      Pose2d robotInTag = toTagFrame(tag.getRobotPose_TargetSpace());
      // The tag's own origin, seen from the robot, added to where the robot was.
      Pose2d tagPose = poseThen.get().plus(new Transform2d(robotInTag, Pose2d.ZERO));
      tagPoses.put(tag.fiducialId, tagPose);
      Telemetry.getTable("Vision").log("RobotInTag" + tag.fiducialId, robotInTag);
    }
  }

  /**
   * Turns a Limelight target-space pose into a flat pose measured from the tag. Target space is the
   * camera's convention: X to the right as you face the tag, Y down, and Z into the wall, so a
   * robot in front of the tag has a negative Z. The result is WPILib's: X straight out of the tag's
   * face, Y to the tag's left, and a heading of 180 degrees when the robot squarely faces the tag.
   */
  private static Pose2d toTagFrame(Pose3d robotInTargetSpace) {
    return new Pose2d(
        -robotInTargetSpace.getZ(),
        robotInTargetSpace.getX(),
        // Turning about the downward Y axis is turning clockwise seen from above.
        Rotation2d.PI.minus(new Rotation2d(robotInTargetSpace.getRotation().getY())));
  }

  /**
   * Where the robot is, measured from tag {@code tagId}: X meters out from its face, Y meters to
   * its left, and the heading relative to the tag. Empty until a camera has seen that tag.
   *
   * <p>Once the tag has been seen, this keeps answering from the wheels after it leaves the frame.
   */
  public Optional<Pose2d> robotInTag(int tagId) {
    Pose2d tagPose = tagPoses.get(tagId);
    if (tagPose == null) {
      return Optional.empty();
    }
    return Optional.of(drivetrain.getPose().relativeTo(tagPose));
  }
}
