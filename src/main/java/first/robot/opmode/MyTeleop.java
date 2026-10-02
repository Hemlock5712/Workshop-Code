// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.opmode;

import first.robot.Robot;
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
public class MyTeleop extends PeriodicOpMode {
  private final CommandNiDsXboxController driver = new CommandNiDsXboxController(0);

  public MyTeleop(Robot robot) {
    // Left trigger: push the arm up while held, stop when released.
    driver.leftTrigger().whileTrue(robot.arm.runFast()).whileFalse(robot.arm.stop());

    // Right trigger: spin fast while held, drop back to the slow voltage when released.
    driver.rightTrigger().whileTrue(robot.flywheel.runFast()).whileFalse(robot.flywheel.runSlow());

    // A: spin fast while held, stop when released.
    driver.a().whileTrue(robot.flywheel.runFast()).whileFalse(robot.flywheel.stop());
  }
}
