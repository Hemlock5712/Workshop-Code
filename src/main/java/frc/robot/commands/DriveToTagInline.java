// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DriveMechanism;
import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.trajectory.TrapezoidProfile;

/**
 * Drives up to an AprilTag using the camera only, no odometry.
 *
 * <p>Inline style: setup, one loop, cleanup, all in one block, where {@link DriveToPoint} uses the
 * four separate {@link frc.robot.utils.ClassicCommand} methods. See frc5712.com.
 */
public final class DriveToTagInline {
  private DriveToTagInline() {}

  /**
   * @param limelightName the camera's name in NetworkTables
   * @param standoffMeters how far in front of the tag to stop
   */
  public static Command create(
      DriveMechanism drivetrain, String limelightName, int targetTagId, double standoffMeters) {
    // One profiled PID per axis: the profile plans a smooth ramp, PID trims the drift.
    // TODO: tune the speed limits and kP on your robot.
    ProfiledPIDController distance =
        new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(2.5, 3.0));
    ProfiledPIDController lateral =
        new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(2.5, 3.0));
    ProfiledPIDController heading =
        new ProfiledPIDController(
            0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(Math.PI, 2.0 * Math.PI));

    SwerveRequest.ApplyRobotVelocity driveRequest =
        new SwerveRequest.ApplyRobotVelocity()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    heading.enableContinuousInput(-Math.PI, Math.PI);
    distance.setTolerance(0.03); // meters
    lateral.setTolerance(0.03); // meters
    heading.setTolerance(Math.toRadians(2.0)); // radians

    return drivetrain
        .run(
            (Coroutine coroutine) -> {
              // Setup. Inside the body, so it re-runs every time the command is scheduled.
              LimelightHelpers.setPriorityTagID(limelightName, targetTagId);

              // Start the profiles from where we are, so we don't lurch. Skipped if no tag yet.
              Pose3d robotInTag = readRobotInTag(limelightName, targetTagId);
              if (robotInTag != null) {
                distance.reset(robotInTag.getZ());
                lateral.reset(robotInTag.getX());
                heading.reset(robotInTag.getRotation().getY());
              }

              // One pass per robot loop.
              while (true) {
                robotInTag = readRobotInTag(limelightName, targetTagId);

                // No tag in sight - hold still and look again next loop.
                if (robotInTag == null) {
                  drivetrain.setControl(new SwerveRequest.Idle());
                  coroutine.yield();
                  continue;
                }

                // Which number is which is explained on readRobotInTag below.
                double measuredDistance = robotInTag.getZ();
                double measuredLateral = robotInTag.getX();
                double measuredYaw = robotInTag.getRotation().getY();

                // Back off to the standoff, slide until centered, turn until square.
                double forward =
                    distance.calculate(measuredDistance, standoffMeters)
                        + distance.getSetpoint().velocity;
                double sideways =
                    lateral.calculate(measuredLateral, 0.0) + lateral.getSetpoint().velocity;
                double turn = heading.calculate(measuredYaw, 0.0) + heading.getSetpoint().velocity;

                // TODO: if the robot slides or turns the wrong way, flip that value's sign.
                drivetrain.setControl(
                    driveRequest.withVelocity(new ChassisVelocities(forward, sideways, turn)));

                // A fresh controller claims to be at its goal, so the null check above matters.
                if (distance.atGoal() && lateral.atGoal() && heading.atGoal()) {
                  break;
                }
                coroutine.yield();
              }

              // Cleanup, on a normal finish.
              drivetrain.setControl(new SwerveRequest.Idle());
              LimelightHelpers.setPriorityTagID(limelightName, -1); // -1 = no priority
            })
        // Being interrupted skips the cleanup above, so repeat it here.
        .whenCanceled(
            () -> {
              drivetrain.setControl(new SwerveRequest.Idle());
              LimelightHelpers.setPriorityTagID(limelightName, -1);
            })
        .named("DriveToTagInline");
  }

  /**
   * The robot's pose in the tag's frame, or null when the camera isn't looking at our tag.
   *
   * <p>Limelight target space: +X is the tag's right, +Y is down, +Z points out of the tag face. So
   * distance is Z, sideways is X, squareness is the rotation about Y. TODO: verify the signs on
   * hardware.
   *
   * <p>The camera reports one tag at a time, so the ID check is what stops us driving at the wrong
   * tag. Don't remove it.
   */
  private static Pose3d readRobotInTag(String limelightName, int targetTagId) {
    // Also false when the camera is unplugged.
    if (!LimelightHelpers.getTV(limelightName)) {
      return null;
    }
    if ((int) LimelightHelpers.getFiducialID(limelightName) != targetTagId) {
      return null;
    }
    Pose3d pose = LimelightHelpers.getBotPose3d_TargetSpace(limelightName);
    // All zeros means no target-space data yet.
    return pose.equals(Pose3d.kZero) ? null : pose;
  }
}
