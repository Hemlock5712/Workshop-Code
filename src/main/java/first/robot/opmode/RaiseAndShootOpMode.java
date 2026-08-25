// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.opmode;

import static org.wpilib.units.Units.Seconds;

import first.robot.Robot;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.PeriodicOpMode;

/**
 * An autonomous routine written as a coroutine: raise the arm, spin up the flywheel, shoot.
 *
 * <p>A coroutine is code that can pause and pick up where it left off, so the routine reads as a
 * plain list of steps. See frc5712.com.
 *
 * <ul>
 *   <li>{@code fork} starts a command and keeps going.
 *   <li>{@code await} waits for a command to finish.
 *   <li>{@code waitUntil} pauses until something is true.
 *   <li>{@code yield} pauses for one loop.
 * </ul>
 */
@Autonomous(name = "Raise And Shoot")
public class RaiseAndShootOpMode extends PeriodicOpMode {
  private final Command routine;

  public RaiseAndShootOpMode(Robot robot) {
    routine =
        Command.noRequirements(
                coroutine -> {
                  // fork, not await: vertical() is a hold and never finishes.
                  coroutine.fork(robot.arm.vertical());

                  // Always time out a wait in an auto, or a stuck arm freezes the whole match.
                  coroutine.await(
                      Command.waitUntil(robot.arm::isAtTarget)
                          .named("wait for the arm")
                          .withTimeout(Seconds.of(3.0))); // TODO: time your own arm

                  // The arm hold is still running here. That is the point of fork.
                  coroutine.fork(robot.flywheel.runFast());
                  coroutine.await(
                      Command.waitUntil(robot.flywheel::isAtTarget)
                          .named("wait for the flywheel")
                          .withTimeout(Seconds.of(3.0)));

                  coroutine.wait(Seconds.of(1.0)); // shoot

                  // Ending the routine cancels both forked holds.
                })
            .named("Raise And Shoot");
  }

  @Override
  public void start() {
    Scheduler.getDefault().schedule(routine);
  }

  @Override
  public void end() {
    Scheduler.getDefault().cancel(routine);
  }
}
