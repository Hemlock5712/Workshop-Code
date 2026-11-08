// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.subsystems.DriveMechanism;
import java.util.Set;
import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;

/**
 * Drives the robot straight to a spot on the field. Three PID controllers (X, Y, and heading)
 * compare where the robot is to where it should be, and turn the difference into a velocity. There
 * is no speed ramp yet - the next lesson (6-ProfiledToPoint) adds one.
 *
 * <p>This implements {@link Command} directly. {@link #run} holds the whole command: setup, then a
 * loop that yields once per robot loop. {@link #name} and {@link #requirements} tell the scheduler
 * what to call it and which mechanism it owns, and {@link #onCancel} stops the drivetrain.
 */
public class DriveToPoint implements Command {
  private final DriveMechanism drivetrain;
  private final Pose2d targetPose;

  // The whole commanded velocity comes from these controllers. The X/Y gain means: meters per
  // second of speed for every meter of error. TODO: tune these for your drivetrain.
  private final PIDController xController = new PIDController(10, 0, 0);
  private final PIDController yController = new PIDController(10, 0, 0);
  private final PIDController headingController = new PIDController(7, 0, 0);

  // A field-relative velocity request. Blue-origin, to match the odometry pose (which is also
  // always blue-origin). Open-loop drive, so there is no wheel PID to tune.
  private final SwerveRequest.ApplyFieldVelocity driveRequest =
      new SwerveRequest.ApplyFieldVelocity()
          .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.BlueAlliance)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  /**
   * @param drivetrain the swerve drive to command
   * @param targetPose the field pose (blue-origin) to drive to, including the goal heading
   */
  public DriveToPoint(DriveMechanism drivetrain, Pose2d targetPose) {
    this.drivetrain = drivetrain;
    this.targetPose = targetPose;

    // Wrap heading error to [-pi, pi] so the robot turns the short way around.
    headingController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void run(Coroutine coroutine) {
    xController.reset();
    yController.reset();
    headingController.reset();

    // Runs until something cancels it (like letting go of the button). To make it stop at the
    // goal instead, loop while this is false, and stop the drivetrain after the loop:
    // xController.atSetpoint() && yController.atSetpoint() && headingController.atSetpoint()
    while (true) {
      Pose2d currentPose = drivetrain.getPose();

      double vx = xController.calculate(currentPose.getX(), targetPose.getX());
      double vy = yController.calculate(currentPose.getY(), targetPose.getY());
      double omega =
          headingController.calculate(
              currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

      drivetrain.setControl(driveRequest.withVelocity(new ChassisVelocities(vx, vy, omega)));
      coroutine.yield();
    }
  }

  /** Stops the drivetrain. Runs when the command is canceled before it finishes. */
  @Override
  public void onCancel() {
    // Zero speed, not SwerveRequest.Idle: Idle leaves every module on its last request.
    drivetrain.setControl(driveRequest.withVelocity(new ChassisVelocities()));
  }

  @Override
  public String name() {
    return "DriveToPoint";
  }

  @Override
  public Set<Mechanism> requirements() {
    return Set.of(drivetrain);
  }
}
