// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.opmode;

import static org.wpilib.units.Units.Seconds;

import first.robot.Robot;
import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Scheduler;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.PeriodicOpMode;

/**
 * The same raise-then-shoot idea as the Y button, but in autonomous.
 *
 * <p>Nobody is holding a button here. So every wait needs a time limit, and this OpMode has to
 * schedule and cancel the routine itself instead of letting a trigger do it. See frc5712.com.
 */
@Autonomous(name = "Raise And Shoot")
public class RaiseAndShootOpMode extends PeriodicOpMode {
  private final Robot robot;
  private final Command routine;

  public RaiseAndShootOpMode(Robot robot) {
    this.robot = robot;
    routine =
        Command.noRequirements(coroutine -> raiseAndShoot(coroutine)).named("Raise And Shoot");
  }

  /**
   * Raise the arm, spin up the flywheel, shoot.
   *
   * <p>Every wait is bounded. A stuck arm ends the routine instead of eating the whole match.
   *
   * @param coroutine the coroutine this routine runs on, handed in by the scheduler
   */
  private void raiseAndShoot(Coroutine coroutine) {
    // fork, not await: vertical() is a hold and never finishes.
    coroutine.fork(robot.arm.vertical());

    // TODO: time your own arm.
    if (coroutine.waitUntil(() -> robot.arm.isAtTarget(), Seconds.of(3.0)).timedOut()) {
      return;
    }

    // The arm hold is still running. That is the point of fork.
    coroutine.fork(robot.flywheel.runFast());

    if (coroutine.waitUntil(() -> robot.flywheel.isAtTarget(), Seconds.of(3.0)).timedOut()) {
      return;
    }

    coroutine.wait(Seconds.of(1.0)); // shoot

    // Returning cancels both forked holds.
  }

  /** No trigger owns this routine, so the OpMode starts and stops it. */
  @Override
  public void start() {
    Scheduler.getDefault().schedule(routine);
  }

  @Override
  public void end() {
    Scheduler.getDefault().cancel(routine);
  }
}
