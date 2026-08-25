// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.opmode;

import first.robot.Robot;
import first.robot.mechanisms.Arm;
import first.robot.mechanisms.Flywheel;
import org.wpilib.command3.button.CommandNiDsXboxController;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.opmode.Teleop;

/**
 * The driver's controls. The framework builds this class when "Teleop" is picked on the driver
 * station. The button bindings made in the constructor belong to this OpMode, and the framework
 * removes them on a mode switch. No cleanup code needed.
 *
 * <p>The buttons here run the arm and flywheel commands.
 */
@Teleop(name = "Teleop")
public class TeleopOpMode extends PeriodicOpMode {
  private final CommandNiDsXboxController driver = new CommandNiDsXboxController(0);

  public TeleopOpMode(Robot robot) {
    final Arm arm = robot.arm;
    final Flywheel flywheel = robot.flywheel;

    // Left trigger: push the arm up while held, stop when released.
    driver.leftTrigger().onTrue(arm.runFast()).onFalse(arm.stop());

    // Right trigger: spin fast while held, drop back to the slow voltage when released.
    driver.rightTrigger().onTrue(flywheel.runFast()).onFalse(flywheel.runSlow());

    // A: spin fast while held, stop when released.
    driver.a().onTrue(flywheel.runFast()).onFalse(flywheel.stop());
  }
}
