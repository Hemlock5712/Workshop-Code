// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.opmodes;

import frc.robot.Robot;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.opmode.Teleop;

/**
 * The driver's buttons. The framework builds this class when "Teleop" is picked on the driver
 * station. The button bindings made in the constructor belong to this OpMode, and the framework
 * removes them on a mode switch. No cleanup code needed.
 *
 * <p>The joystick drive itself is not here. {@link Robot} sets it as the drivetrain's default
 * command, so it applies in every mode. Add a second {@code @Teleop} class and it shows up as
 * another choice on the driver station.
 */
@Teleop(name = "Teleop")
public class TeleopOpMode extends PeriodicOpMode {
  public TeleopOpMode(Robot robot) {
    // Left bumper: make the robot's current facing the new "forward".
    robot.driver.leftBumper().onTrue(robot.drivetrain.seedFieldCentric());

    // Hold A or B to drive straight to a fixed spot on the field. Let go to stop.
    robot.driver.a().whileTrue(robot.drivetrain.driveToPoint(Pose2d.ZERO));
    robot.driver.b().whileTrue(robot.drivetrain.driveToPoint(new Pose2d(3, 2, Rotation2d.PI)));

    // Hold X to line up 1 m straight out from tag 7, facing it. The spot is measured from the
    // tag, so the field pose can be wrong and the robot still stops in the same place.
    robot
        .driver
        .x()
        .whileTrue(
            robot.drivetrain.alignToTag(robot.vision, 7, new Pose2d(1.0, 0.0, Rotation2d.PI)));
  }
}
