// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.opmodes;

import frc.robot.Robot;
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

    // Right trigger: spin up for the distance to the goal while held, stop when released.
    robot
        .driver
        .rightTrigger()
        .whileTrue(robot.flywheel.distanceShoot())
        .whileFalse(robot.flywheel.stop());
  }
}
