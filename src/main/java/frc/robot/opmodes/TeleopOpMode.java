// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.opmodes;

import frc.robot.Robot;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Flywheel;
import org.wpilib.command3.button.CommandNiDsXboxController;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.opmode.Teleop;

/**
 * The driver's controls. The framework builds this class when "Teleop" is picked on the driver
 * station. The button bindings made in the constructor belong to this OpMode, and the framework
 * removes them on a mode switch. No cleanup code needed.
 *
 * <p>The buttons here run the arm and flywheel PID commands.
 */
@Teleop(name = "Teleop")
public class TeleopOpMode extends PeriodicOpMode {
  private final CommandNiDsXboxController driver = new CommandNiDsXboxController(0);

  public TeleopOpMode(Robot robot) {
    final Arm arm = robot.arm;
    final Flywheel flywheel = robot.flywheel;

    // Hold the left trigger to drive the arm to its vertical position (and hold it there).
    driver.leftTrigger().onTrue(arm.vertical());

    // Right trigger: spin fast while held, drop back to the slow hold speed when released.
    driver.rightTrigger().onTrue(flywheel.runFast()).onFalse(flywheel.runSlow());

    // A: spin fast while held, stop when released.
    driver.a().onTrue(flywheel.runFast()).onFalse(flywheel.stop());
  }
}
