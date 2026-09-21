// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.opmode;

import first.robot.Robot;
import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.command3.button.CommandNiDsXboxController;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.opmode.Teleop;

/**
 * The driver's controls. The framework builds this class when "Teleop" is picked on the driver
 * station. The button bindings made in the constructor belong to this OpMode, and the framework
 * removes them on a mode switch. No cleanup code needed.
 *
 * <p>The buttons here run the arm and flywheel PID commands. New in this lesson: the Y button waits
 * for the arm to really arrive before spinning up the flywheel.
 */
@Teleop(name = "Teleop")
public class MyTeleop extends PeriodicOpMode {
  private final CommandNiDsXboxController driver = new CommandNiDsXboxController(0);
  private final Robot robot;

  public MyTeleop(Robot robot) {
    this.robot = robot;
    // Hold the left trigger to drive the arm to its vertical position. Releasing cancels the
    // command; the position request stays applied, so the arm holds where it is.
    driver.leftTrigger().whileTrue(robot.arm.vertical());

    // Right trigger: spin fast while held, drop back to the slow hold speed when released.
    driver.rightTrigger().whileTrue(robot.flywheel.runFast()).whileFalse(robot.flywheel.runSlow());

    // A: spin fast while held, stop when released.
    driver.a().whileTrue(robot.flywheel.runFast()).whileFalse(robot.flywheel.stop());

    // Y: raise the arm, then spin the flywheel once it is really there.
    driver
        .y()
        .whileTrue(
            Command.noRequirements(coroutine -> spinUpWhenReady(coroutine))
                .named("Spin Up When Ready (hold)"))
        .whileFalse(robot.flywheel.stop());
  }

  /**
   * Raise the arm, wait for it to get there, then spin the flywheel and keep it spinning.
   *
   * <p>A coroutine rather than Command.sequence, which would own both mechanisms for the whole run
   * while only driving one at a time. Here the routine requires nothing and each step requires just
   * its own mechanism.
   *
   * <p>No time limit on the wait, because the driver can let go. Lesson 5 does this in an auto,
   * where nobody can.
   *
   * @param coroutine the coroutine this routine runs on, handed in by the scheduler
   */
  private void spinUpWhenReady(Coroutine coroutine) {
    // fork, not await: vertical() is a hold and never finishes.
    coroutine.fork(robot.arm.vertical());

    coroutine.waitUntil(() -> robot.arm.isAtTarget());

    // runFast is a hold too, so this never returns: releasing Y cancels the whole routine.
    coroutine.await(robot.flywheel.runFast());
  }
}
