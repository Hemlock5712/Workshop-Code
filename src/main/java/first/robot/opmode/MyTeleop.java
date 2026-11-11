// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.opmode;

import static org.wpilib.units.Units.Seconds;

import first.robot.Robot;
import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.opmode.Teleop;

/**
 * The driver's controls. The framework builds this class when "Teleop" is picked on the driver
 * station. The button bindings made in the constructor belong to this OpMode, and the framework
 * removes them on a mode switch. No cleanup code needed.
 *
 * <p>The triggers and A run the arm and flywheel commands. New in this lesson: Y, X and B each run
 * a routine, a coroutine that starts a command, waits for the mechanism to arrive, and goes on.
 */
@Teleop(name = "Teleop")
public class MyTeleop extends PeriodicOpMode {
  private final Robot robot;

  public MyTeleop(Robot robot) {
    this.robot = robot;
    // Hold the left trigger to drive the arm to its vertical position. Releasing cancels the
    // command; the position request stays applied, so the arm finishes the move and holds there.
    robot.driver.leftTrigger().whileTrue(robot.arm.vertical());

    // Right trigger: spin fast while held, drop back to the slow hold speed when released.
    robot
        .driver
        .rightTrigger()
        .whileTrue(robot.flywheel.runFast())
        .whileFalse(robot.flywheel.runSlow());

    // A: spin fast while held, stop when released.
    robot.driver.a().whileTrue(robot.flywheel.runFast()).whileFalse(robot.flywheel.stop());

    // Y: raise the arm, wait for it, lower it, wait again.
    robot
        .driver
        .y()
        .whileTrue(
            Command.noRequirements(coroutine -> raiseAndLower(coroutine)).named("Raise And Lower"));

    // X: spin up, keep the speed for one second, stop.
    robot
        .driver
        .x()
        .whileTrue(
            Command.noRequirements(coroutine -> spinUpAndStop(coroutine)).named("Spin Up And Stop"))
        .whileFalse(robot.flywheel.stop());

    // B: raise and lower the arm three times.
    robot.driver.b().whileTrue(Command.noRequirements(coroutine -> wave(coroutine)).named("Wave"));
  }

  /**
   * Raise the arm, wait for it to arrive, then lower it and wait again.
   *
   * <p>vertical() and horizontal() use runRepeatedly, so they never finish. fork starts each one
   * and moves on. No time limit on the waits, because the driver can let go.
   *
   * @param coroutine the coroutine this routine runs on, handed in by the scheduler
   */
  private void raiseAndLower(Coroutine coroutine) {
    coroutine.fork(robot.arm.vertical());
    coroutine.waitUntil(() -> robot.arm.isAtTarget());

    coroutine.fork(robot.arm.horizontal());
    coroutine.waitUntil(() -> robot.arm.isAtTarget());

    System.out.println("Raise And Lower: done");
  }

  /**
   * Spin the flywheel up, keep the speed for one second, then stop it.
   *
   * @param coroutine the coroutine this routine runs on, handed in by the scheduler
   */
  private void spinUpAndStop(Coroutine coroutine) {
    coroutine.fork(robot.flywheel.runFast());
    coroutine.waitUntil(() -> robot.flywheel.isAtTarget());

    coroutine.wait(Seconds.of(1.0));

    // stop() replaces runFast() and sends its zero before the routine ends.
    coroutine.fork(robot.flywheel.stop());
    System.out.println("Spin Up And Stop: done");
  }

  /**
   * Raise and lower the arm three times. A routine is ordinary Java, so a for loop repeats it.
   *
   * @param coroutine the coroutine this routine runs on, handed in by the scheduler
   */
  private void wave(Coroutine coroutine) {
    for (int i = 0; i < 3; i++) {
      coroutine.fork(robot.arm.vertical());
      coroutine.waitUntil(() -> robot.arm.isAtTarget());

      coroutine.fork(robot.arm.horizontal());
      coroutine.waitUntil(() -> robot.arm.isAtTarget());
    }
    System.out.println("Wave: done");
  }
}
