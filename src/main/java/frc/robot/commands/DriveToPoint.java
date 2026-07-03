// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.subsystems.DriveMechanism;
import frc.robot.utils.ClassicCommand;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;

/**
 * Drives the robot straight to a spot on the field. Three PID controllers (X, Y, and heading)
 * compare where the robot is to where it should be, and turn the difference into a velocity. There
 * is no speed ramp yet - the next lesson (6-ProfiledToPoint) adds one.
 *
 * <p>This extends {@link ClassicCommand}, so it is written in four familiar steps: initialize,
 * execute, isFinished, end. {@code super("DriveToPoint", drivetrain)} sets the command's name and
 * marks the drivetrain as required.
 */
public class DriveToPoint extends ClassicCommand {
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
    super("DriveToPoint", drivetrain); // command name + required mechanism
    this.drivetrain = drivetrain;
    this.targetPose = targetPose;

    // Wrap heading error to [-pi, pi] so the robot turns the short way around.
    headingController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  protected void initialize() {
    xController.reset();
    yController.reset();
    headingController.reset();
  }

  @Override
  protected void execute() {
    Pose2d currentPose = drivetrain.getPose();

    double vx = xController.calculate(currentPose.getX(), targetPose.getX());
    double vy = yController.calculate(currentPose.getY(), targetPose.getY());
    double omega =
        headingController.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    drivetrain.setControl(driveRequest.withVelocity(new ChassisVelocities(vx, vy, omega)));
  }

  @Override
  protected boolean isFinished() {
    // Runs until something interrupts it (like letting go of the button). To make it stop at
    // the goal instead, return:
    // xController.atSetpoint() && yController.atSetpoint() && headingController.atSetpoint();
    return false;
  }

  /** Stops the drivetrain. Runs when the command finishes or gets interrupted. */
  @Override
  protected void end(boolean interrupted) {
    drivetrain.setControl(new SwerveRequest.Idle());
  }
}
