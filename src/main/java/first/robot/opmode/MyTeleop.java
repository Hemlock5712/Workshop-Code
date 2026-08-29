// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.opmode;

import first.robot.Robot;
import org.wpilib.command3.Command;
import org.wpilib.command3.button.CommandNiDsXboxController;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.opmode.Teleop;

/**
 * The driver's controls. The framework builds this class when "Teleop" is picked on the driver
 * station. The button bindings made in the constructor belong to this OpMode, and the framework
 * removes them on a mode switch. No cleanup code needed.
 *
 * <p>The buttons here run the arm and flywheel PID commands. New in this lesson: the Y button
 * builds a sequence that waits for the arm to really arrive before spinning up the flywheel.
 */
@Teleop(name = "Teleop")
public class MyTeleop extends PeriodicOpMode {
  private final CommandNiDsXboxController driver = new CommandNiDsXboxController(0);

  public MyTeleop(Robot robot) {
    // Hold the left trigger to drive the arm to its vertical position. Releasing cancels the
    // command; the position request stays applied, so the arm holds where it is.
    driver.leftTrigger().whileTrue(robot.arm.vertical());

    // Right trigger: spin fast while held, drop back to the slow hold speed when released.
    driver.rightTrigger().whileTrue(robot.flywheel.runFast()).whileFalse(robot.flywheel.runSlow());

    // A: spin fast while held, stop when released.
    driver.a().whileTrue(robot.flywheel.runFast()).whileFalse(robot.flywheel.stop());

    // Y: raise the arm, wait until it really reaches the target, then spin the flywheel fast.
    // Without isAtTarget there is no way to know when "then" is.
    //
    // vertical() is a hold, so it never finishes. Dropped straight into Command.sequence it would
    // stick there forever. .until(arm::isAtTarget) gives the hold an ending right here. Do not
    // add a special "AndWait" method to the mechanism. In an auto, also add .withTimeout(seconds)
    // as a time limit. If the arm never quite arrives, the routine moves on instead of getting
    // stuck for the rest of the period.
    //
    // The last step (runFast) is still a hold, so the whole sequence is a hold too. That is why
    // its name ends in "(hold)" and why we use whileTrue: releasing Y cancels it.
    driver
        .y()
        .whileTrue(
            Command.sequence(
                    robot.arm
                        .vertical()
                        .until(robot.arm::isAtTarget)
                        .named("vertical until at target"),
                    robot.flywheel.runFast())
                .named("Spin Up When Ready (hold)"))
        .whileFalse(robot.flywheel.stop());
  }
}
