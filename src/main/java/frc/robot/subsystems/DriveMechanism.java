// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static org.wpilib.units.Units.Seconds;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import frc.robot.generated.TunerConstants;
import java.util.function.Supplier;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
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

  // The mass, MOI and module layout typed into the PathPlanner app's Robot Config page. The app
  // saves them to deploy/pathplanner/settings.json, and this reads that file.
  private final RobotConfig pathConfig = loadPathConfig();

  // Pulls the robot back onto the path when it drifts. The first gain is for position (m/s of
  // correction per meter of error), the second for heading. TODO: tune on your robot.
  private final PPHolonomicDriveController pathController =
      new PPHolonomicDriveController(
          new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0));

  // PathPlanner hands back speeds relative to the robot, so the request is robot-relative too.
  private final SwerveRequest.ApplyRobotVelocity pathRequest =
      new SwerveRequest.ApplyRobotVelocity();

  // Speed limits for routes the pathfinder makes up: 2 m/s, 2 m/s², and a turn rate of 3/4 of a
  // turn per second. A drawn path carries its own limits; a found one gets these.
  private final PathConstraints pathfindConstraints =
      new PathConstraints(2.0, 2.0, Math.toRadians(270), Math.toRadians(360), 12.0);

  public DriveMechanism() {
    // Load deploy/pathplanner/navgrid.json and start the route search on its own thread, now,
    // so the first request does not pay for it.
    Pathfinding.ensureInitialized();
    // Every loop, check which alliance we are on so "forward" faces the right way.
    Scheduler.getDefault().addPeriodic(() -> drivetrain.applyOperatorPerspective());
    // CTRE feeds this fresh data up to 250 times per second, from its own thread.
    // Telemetry is safe to call from any thread.
    drivetrain.registerTelemetry(state -> logState(state));
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
   * Returns a command that drives one PathPlanner path from start to finish, then stops.
   *
   * <p>The plan is made when the command starts, from wherever the robot is and however fast it is
   * moving. Each loop asks the plan where the robot should be right now, and the controller turns
   * the gap into a speed.
   *
   * @param path a path loaded with {@link #loadPath(String)}
   */
  public Command followPath(PathPlannerPath path) {
    return run(coroutine -> {
          PathPlannerTrajectory trajectory =
              path.generateTrajectory(getRobotVelocity(), getPose().getRotation(), pathConfig);
          pathController.reset(getPose(), getRobotVelocity());
          double startTime = Utils.getCurrentTimeSeconds();
          double elapsed = 0.0;

          while (elapsed < trajectory.getTotalTimeSeconds()) {
            PathPlannerTrajectoryState target = trajectory.sample(elapsed);
            drivetrain.setControl(
                pathRequest.withVelocity(
                    pathController.calculateRobotRelativeSpeeds(getPose(), target)));
            Telemetry.getTable(getName()).log("PathTarget", target.pose);
            coroutine.yield();
            elapsed = Utils.getCurrentTimeSeconds() - startTime;
          }

          // setControl latches. Without this, the path's last speed keeps being applied.
          stopDriving();
          System.out.println(
              +elapsed
                  + " total="
                  + trajectory.getTotalTimeSeconds()
                  + " pose="
                  + getPose()
                  + " target="
                  + trajectory.getEndState().pose);
        })
        .whenCanceled(() -> stopDriving())
        .named("FollowPath " + path.name);
  }

  /**
   * Returns a command that finds a route around the field's obstacles to {@code goal}, then drives
   * it and stops.
   *
   * <p>The route is planned once, from where the robot is when the command starts. The obstacles
   * are the blocked cells in deploy/pathplanner/navgrid.json, which the PathPlanner app edits.
   *
   * @param goal where to end up, blue-origin, including the heading to finish at
   */
  public Command pathfindTo(Pose2d goal) {
    return run(coroutine -> {
          Pathfinding.setStartPosition(getPose().getTranslation());
          Pathfinding.setGoalPosition(goal.getTranslation());

          // The search runs on another thread. No route in a second means none is coming.
          if (coroutine
              .waitUntil(() -> Pathfinding.isNewPathAvailable(), Seconds.of(1.0))
              .timedOut()) {
            stopDriving();
            return;
          }

          PathPlannerPath route =
              Pathfinding.getCurrentPath(
                  pathfindConstraints, new GoalEndState(0.0, goal.getRotation()));
          if (route == null) {
            stopDriving(); // the search found no route
            return;
          }

          coroutine.await(followPath(route));
        })
        .whenCanceled(() -> stopDriving())
        .named("PathfindTo");
  }

  /** Sends zero speed. The drivetrain holds still until something sends another request. */
  public void stopDriving() {
    drivetrain.setControl(pathRequest.withVelocity(new ChassisVelocities()));
  }

  /**
   * Tells odometry where the robot is. An autonomous routine calls this once, before its first
   * path, with the pose that path starts from.
   */
  public void resetPose(Pose2d pose) {
    drivetrain.resetPose(pose);
  }

  /** How fast the robot is moving, in its own directions. X is the robot's forward. */
  public ChassisVelocities getRobotVelocity() {
    return drivetrain.getState().Velocity;
  }

  /**
   * Loads a path the PathPlanner app saved to deploy/pathplanner/paths.
   *
   * @param pathName the name shown in the app, without ".path"
   */
  public static PathPlannerPath loadPath(String pathName) {
    try {
      return PathPlannerPath.fromPathFile(pathName);
    } catch (Exception e) {
      throw new IllegalStateException("Could not load PathPlanner path \"" + pathName + "\"", e);
    }
  }

  private static RobotConfig loadPathConfig() {
    try {
      return RobotConfig.fromGUISettings();
    } catch (Exception e) {
      throw new IllegalStateException("Could not load deploy/pathplanner/settings.json", e);
    }
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
