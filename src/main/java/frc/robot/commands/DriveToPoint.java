// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.LinearPath;
import frc.robot.subsystems.DriveMechanism;
import frc.robot.utils.ClassicCommand;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.trajectory.TrapezoidProfile;

/**
 * Drives the robot in a straight line to a spot on the field, with a smooth speed ramp.
 *
 * <p>This is the upgraded 5-DriveToPoint. That version turned raw PID output into wheel speed, so
 * it lurched. This version plans the whole trip first - speed up, cruise, slow down - using CTRE's
 * {@link LinearPath}, and then follows the plan. It turns to the goal heading along the way, and it
 * is a good building block for an autonomous routine.
 *
 * <p>Each loop, the command asks the plan "where should I be right now, and how fast?" The planned
 * velocity does most of the driving. The three PID controllers (X, Y, heading) only trim off the
 * small drift between where odometry says we are and where the plan says we should be.
 *
 * <p>Still a {@link ClassicCommand}: initialize, execute, isFinished, end.
 */
public class DriveToPoint extends ClassicCommand {
  private final DriveMechanism drivetrain;
  private final Pose2d goal;

  // The trip planner. First pair of limits: top speed (m/s) and acceleration (m/s²) for driving.
  // Second pair: the same for turning (rad/s, rad/s²). TODO: tune to what your drivetrain can do.
  private final LinearPath path =
      new LinearPath(
          new TrapezoidProfile.Constraints(2.5, 3.0),
          new TrapezoidProfile.Constraints(Math.PI, 2.0 * Math.PI));

  // Drift correction, one controller per axis. The plan's velocity does the driving; these only
  // nudge the robot back onto the plan. Raise kP if the robot lags the plan or stops short of
  // the goal. Lower it (or add kD) if the robot wobbles. TODO: tune.
  private final PIDController xController = new PIDController(3.0, 0.0, 0.0);
  private final PIDController yController = new PIDController(3.0, 0.0, 0.0);
  private final PIDController headingController = new PIDController(4.0, 0.0, 0.0);

  // A field-relative velocity request. Blue-origin, to match the odometry pose (which is also
  // always blue-origin). Open-loop drive, so there is no wheel PID to tune.
  private final SwerveRequest.ApplyFieldVelocity driveRequest =
      new SwerveRequest.ApplyFieldVelocity()
          .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.BlueAlliance)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  // Where the robot was, and how fast it was moving, when the command started. The whole trip is
  // planned from this one snapshot.
  private LinearPath.State startState = new LinearPath.State();
  // When the command started. (now - startTime) says how far into the trip we are.
  private double startTime;

  /**
   * @param drivetrain the swerve drive to command
   * @param goal the field pose (blue-origin) to drive to, including the goal heading
   */
  public DriveToPoint(DriveMechanism drivetrain, Pose2d goal) {
    super("DriveToPoint", drivetrain); // command name + required mechanism
    this.drivetrain = drivetrain;
    this.goal = goal;
    headingController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /** Takes the starting snapshot and starts the trip clock. */
  @Override
  protected void initialize() {
    startState = new LinearPath.State(drivetrain.getPose(), drivetrain.getFieldVelocity());
    startTime = Utils.getCurrentTimeSeconds();
    xController.reset();
    yController.reset();
    headingController.reset();
  }

  /** Runs every robot loop while the command is active. */
  @Override
  protected void execute() {
    // Ask the plan where we should be, this many seconds into the trip.
    double t = Utils.getCurrentTimeSeconds() - startTime;
    LinearPath.State setpoint = path.calculate(t, startState, goal);

    Pose2d measuredPose = drivetrain.getPose();

    // The plan's velocity does the driving. Each PID call below adds a small correction that
    // pulls the measured pose back onto the planned pose.
    ChassisVelocities feedforward = setpoint.velocity;
    double vx = feedforward.vx + xController.calculate(measuredPose.getX(), setpoint.pose.getX());
    double vy = feedforward.vy + yController.calculate(measuredPose.getY(), setpoint.pose.getY());
    double omega =
        feedforward.omega
            + headingController.calculate(
                measuredPose.getRotation().getRadians(), setpoint.pose.getRotation().getRadians());

    drivetrain.setControl(driveRequest.withVelocity(new ChassisVelocities(vx, vy, omega)));
  }

  /**
   * Done when the planned trip time is up. The plan knows how long the whole trip takes, so "am I
   * done" is just a time check - no position tolerances needed.
   */
  @Override
  protected boolean isFinished() {
    return path.isFinished(Utils.getCurrentTimeSeconds() - startTime);
  }

  /** Stops the drivetrain. Runs when the command finishes or gets interrupted. */
  @Override
  protected void end(boolean interrupted) {
    drivetrain.setControl(new SwerveRequest.Idle());
  }
}
