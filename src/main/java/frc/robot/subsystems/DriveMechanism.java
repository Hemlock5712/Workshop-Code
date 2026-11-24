// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import bline.commands.FollowPath;
import bline.following.DriveType;
import bline.path.Path;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.generated.TunerConstants;
import java.util.function.Supplier;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.telemetry.TelemetryTable;

/**
 * The drivetrain's {@code Mechanism}. The swerve drivetrain class already extends CTRE's generated
 * class, and a Java class can only extend one thing. So this class owns the drivetrain and offers
 * its commands to the rest of the robot.
 */
public class DriveMechanism implements Mechanism {
  // TunerConstants comes from the Tuner X swerve generator. The checked-in file is an EXAMPLE
  // with fake device IDs and gains - regenerate it from Tuner X for your own robot.
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  // BLine hands back speeds relative to the robot, so the request is robot-relative too.
  // Velocity asks each wheel for an exact speed, so the small corrections at the end of a path
  // still turn the wheels instead of stalling against friction.
  private final SwerveRequest.ApplyRobotVelocity pathRequest =
      new SwerveRequest.ApplyRobotVelocity().withDriveRequestType(DriveRequestType.Velocity);

  // Turns a path file into a command that drives it. Built once, used for every path.
  private final FollowPath.Builder pathBuilder;

  public DriveMechanism() {
    // Every loop, check which alliance we are on so "forward" faces the right way.
    Scheduler.getDefault().addPeriodic(() -> drivetrain.applyOperatorPerspective());
    // CTRE feeds this fresh data up to 250 times per second, from its own thread.
    // Telemetry is safe to call from any thread.
    drivetrain.registerTelemetry(state -> logState(state));

    // Teaches BLine how to drive this robot. After this, followPath can turn any path drawn in
    // the BLine editor into a command.
    pathBuilder =
        new FollowPath.Builder(
                DriveType.SWERVE,
                this, // path commands require this mechanism
                () -> getPose(), // where the robot is
                pose -> resetPose(pose), // used when a path says where the robot starts
                () -> getRobotVelocity(), // how fast it is moving, in its own directions
                velocities -> driveRobotRelative(velocities), // drive like this
                // Each controller pulls the robot back toward the path. TODO: tune on your robot.
                new PIDController(2.0, 0.0, 0.0), // distance to the target: m/s per meter
                // At 1.0 the turn lagged and stalled short of the end tolerance in simulation.
                new PIDController(3.0, 0.0, 0.0), // heading error: rad/s per radian
                new PIDController(0.2, 0.0, 0.0)) // drift off the line: m/s per meter
            // Paths are drawn from the blue side. On red, BLine turns them around the field center.
            .withDefaultShouldFlip();
  }

  /** The mechanism's name. Commands and the telemetry table are both named after it. */
  @Override
  public String getName() {
    return "Drivetrain";
  }

  /**
   * Publishes one drivetrain state. Everything lands under {@code Drivetrain/}, which you can watch
   * live in AdvantageScope or Elastic. Want to watch something else? Add a line.
   *
   * <p>AdvantageScope's swerve widget reads ModuleStates, ModuleTargets and ModulePositions
   * directly.
   */
  private void logState(SwerveDriveState state) {
    TelemetryTable table = Telemetry.getTable(getName());

    table.log("Pose", state.Pose);
    table.log("Velocity", state.Velocity);
    table.log("RawHeading", state.RawHeading);

    table.log("ModuleStates", state.ModuleVelocities);
    table.log("ModuleTargets", state.ModuleTargets);
    table.log("ModulePositions", state.ModulePositions);

    table.log("TranslationSpeedMps", Math.hypot(state.Velocity.vx, state.Velocity.vy));
    table.log("RotationSpeedRadPerSec", state.Velocity.omega);
    table.log("OdometryPeriodSeconds", state.OdometryPeriod);
    table.log("OdometryFrequencyHz", state.OdometryPeriod > 0 ? 1.0 / state.OdometryPeriod : 0.0);
  }

  /** Returns a command that keeps sending the given control request to the drivetrain. */
  public Command applyRequest(Supplier<SwerveRequest> request) {
    return runRepeatedly(() -> drivetrain.setControl(request.get())).named("applyRequest");
  }

  /** Resets the field-centric heading so "forward" matches the driver's current facing. */
  public Command seedFieldCentric() {
    return run(coroutine -> {
          drivetrain.seedFieldCentric();
        })
        .named("seedFieldCentric");
  }

  /**
   * Sends one control request straight to the drivetrain. A command that already requires this
   * mechanism (like {@code DriveToPoint} in a later lesson) uses this to drive.
   */
  public void setControl(SwerveRequest request) {
    drivetrain.setControl(request);
  }

  /**
   * Returns a command that drives the path saved as {@code deploy/autos/paths/<name>.json}. The
   * file is read here, so a missing or misspelled path fails when the OpMode is picked, not halfway
   * through auto.
   *
   * <p>The command sends a zero when the path finishes and when it is canceled partway.
   */
  public FollowPath followPath(String name) {
    return pathBuilder.build(new Path(name));
  }

  /** Drives at the given speeds, in the robot's own directions. X is the robot's forward. */
  public void driveRobotRelative(ChassisVelocities velocities) {
    drivetrain.setControl(pathRequest.withVelocity(velocities));
  }

  /**
   * Tells odometry where the robot is. A path command with a pose reset calls this at its start,
   * with the pose the path starts from.
   */
  public void resetPose(Pose2d pose) {
    drivetrain.resetPose(pose);
  }

  /** How fast the robot is moving, in its own directions. X is the robot's forward. */
  public ChassisVelocities getRobotVelocity() {
    return drivetrain.getState().Velocity;
  }

  /**
   * The robot's position on the field, from odometry. (0, 0) is always the blue alliance corner. It
   * does not flip when you are on red.
   */
  public Pose2d getPose() {
    return drivetrain.getState().Pose;
  }

  /**
   * How fast the robot is moving across the field. The drivetrain measures speed relative to the
   * robot, so this turns it into field directions.
   */
  public ChassisVelocities getFieldVelocity() {
    var state = drivetrain.getState();
    return state.Velocity.toFieldRelative(state.Pose.getRotation());
  }

  /**
   * Feeds a camera position estimate into the drivetrain so it can correct odometry. The Limelight
   * subsystem (added in a later lesson) calls this.
   *
   * @param visionRobotPose the robot position the camera measured (blue-alliance origin)
   * @param timestampSeconds when the picture was taken, on the {@code
   *     Utils.getCurrentTimeSeconds()} clock
   * @param stdDevs how much to trust the measurement [x, y, theta]ᵀ (meters, radians) - bigger
   *     numbers mean trust it less
   */
  public void addVisionMeasurement(
      Pose2d visionRobotPose, double timestampSeconds, Matrix<N3, N1> stdDevs) {
    drivetrain.addVisionMeasurement(visionRobotPose, timestampSeconds, stdDevs);
  }
}
