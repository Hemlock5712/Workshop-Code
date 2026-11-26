// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.mechanisms;

import bline.commands.FollowPath;
import bline.following.DriveType;
import bline.path.Path;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import first.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import java.util.function.Supplier;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.system.Notifier;
import org.wpilib.system.RobotController;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.telemetry.TelemetryTable;

/**
 * The swerve drivetrain. It extends the class Tuner X generates, which owns the hardware and
 * odometry, and it is a {@code Mechanism}, so commands can claim it. Your own drivetrain commands
 * go in this file.
 *
 * <p>{@code TunerConstants} builds the one drivetrain with {@code createDrivetrain()}. The
 * checked-in {@code TunerConstants} is an example with made-up device IDs and gains. Generate your
 * own from Tuner X.
 * https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/tuner-swerve/index.html
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Mechanism {
  private static final double SIM_LOOP_PERIOD = 0.004; // 4 ms
  private Notifier simNotifier = null;
  private double lastSimTime;

  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.ZERO;
  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.PI;
  /* True once the operator perspective has been applied at least once */
  private boolean hasAppliedOperatorPerspective = false;

  // BLine hands back speeds relative to the robot, so the request is robot-relative too.
  // Velocity asks each wheel for an exact speed, so the small corrections at the end of a path
  // still turn the wheels instead of stalling against friction.
  private final SwerveRequest.ApplyRobotVelocity pathRequest =
      new SwerveRequest.ApplyRobotVelocity().withDriveRequestType(DriveRequestType.Velocity);

  // Turns a path file into a command that drives it. Built once, used for every path.
  private final FollowPath.Builder pathBuilder;

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param modules Constants for each specific module
   */
  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }

    // Every loop, check which alliance we are on so "forward" faces the right way.
    Scheduler.getDefault().addPeriodic(() -> applyOperatorPerspective());
    // CTRE feeds this fresh data up to 250 times per second, from its own thread.
    // Telemetry is safe to call from any thread.
    registerTelemetry(state -> logState(state));

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

  /** Returns a command that keeps sending the given control request to the drivetrain. */
  public Command applyRequest(Supplier<SwerveRequest> request) {
    return runRepeatedly(() -> setControl(request.get())).named("applyRequest");
  }

  /** Returns a command that makes the robot's current facing the driver's new "forward". */
  public Command resetHeading() {
    return run(coroutine -> {
          seedFieldCentric();
        })
        .named("resetHeading");
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

  /**
   * Returns a command that drives straight to a spot on the field, from wherever the robot is when
   * it starts. It is a BLine path with one waypoint, built in code instead of drawn in the editor,
   * so it gets the same controllers, speed limits and end tolerances as every drawn path.
   *
   * <p>Like a drawn path, the spot is given for the blue alliance. On red, BLine turns it around
   * the field center, so the same button drives to the matching spot on your own side.
   *
   * <p>The command finishes when the robot is within the end tolerances in {@code config.json}, and
   * sends a zero when it finishes or is canceled.
   *
   * @param goal where to drive on blue, in field coordinates from the blue alliance corner
   */
  public FollowPath driveToPoint(Pose2d goal) {
    return pathBuilder.build(new Path(new Path.Waypoint(goal)));
  }

  /** Drives at the given speeds, in the robot's own directions. X is the robot's forward. */
  public void driveRobotRelative(ChassisVelocities velocities) {
    setControl(pathRequest.withVelocity(velocities));
  }

  /**
   * The robot's position on the field, from odometry. (0, 0) is always the blue alliance corner. It
   * does not flip when you are on red.
   */
  public Pose2d getPose() {
    return getState().Pose;
  }

  /** How fast the robot is moving, in its own directions. X is the robot's forward. */
  public ChassisVelocities getRobotVelocity() {
    return getState().Velocity;
  }

  /**
   * How fast the robot is moving across the field. The drivetrain measures speed relative to the
   * robot, so this turns it into field directions.
   */
  public ChassisVelocities getFieldVelocity() {
    var state = getState();
    return state.Velocity.toFieldRelative(state.Pose.getRotation());
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

  /** Keeps "forward" matched to the alliance color. Runs every loop. */
  private void applyOperatorPerspective() {
    /*
     * Apply it right away if we never have (this covers a code restart mid-match). After that,
     * only re-apply while disabled, so the driving feel never changes while someone is driving.
     */
    if (!hasAppliedOperatorPerspective || RobotState.isDisabled()) {
      MatchState.getAlliance()
          .ifPresent(
              allianceColor -> {
                setOperatorForwardDirection(
                    allianceColor == Alliance.RED
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation);
                hasAppliedOperatorPerspective = true;
              });
    }
  }

  private void startSimThread() {
    lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - lastSimTime;
              lastSimTime = currentTime;

              /* use the measured time delta, get battery voltage from WPILib */
              updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });
    simNotifier.startPeriodic(SIM_LOOP_PERIOD);
  }
}
