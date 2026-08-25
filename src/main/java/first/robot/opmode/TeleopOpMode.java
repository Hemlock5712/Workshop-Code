// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.opmode;

import first.robot.Robot;
import first.robot.mechanisms.Arm;
import first.robot.mechanisms.Flywheel;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.StateMachine;
import org.wpilib.command3.StateMachine.State;
import org.wpilib.command3.button.CommandNiDsXboxController;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.opmode.Teleop;
import org.wpilib.system.DataLogManager;

/**
 * State-based teleop. The state machine lesson at <a
 * href="https://frc5712.com/state-based">frc5712.com/state-based</a>, as real code.
 *
 * <p>In the last lesson, each button started and stopped its own commands. Here the robot is always
 * in exactly one named state, either stowed, pickup, spin-up or ready, and the buttons move it
 * between states. Each state owns one command. The {@link StateMachine} cancels the old state's
 * command and starts the new one for you, and the robot can never jump to a state you did not
 * connect with a transition.
 *
 * <p>Building one takes four steps, numbered in the constructor below: build the machine, add
 * states, pick the starting state, and wire the transitions.
 */
@Teleop(name = "Teleop")
public class TeleopOpMode extends PeriodicOpMode {
  private final CommandNiDsXboxController driver = new CommandNiDsXboxController(0);
  private final Command machine;

  public TeleopOpMode(Robot robot) {
    final Arm arm = robot.arm;
    final Flywheel flywheel = robot.flywheel;

    // 1. Build the machine. The name is required and shows up in telemetry.
    StateMachine sm = new StateMachine("Superstructure");

    // 2. Add states. Each state owns one command. parallel(...) turns two commands into one, so
    //    each state poses the whole robot. The poses are holds that never finish, which is fine
    //    because the machine cancels the old state's command when it switches. SpinUp is the
    //    exception: .until(...) gives its command an ending, so it can use whenComplete() below.
    State stowed =
        sm.addState(Command.parallel(arm.vertical(), flywheel.stop()).named("Stowed (hold)"));
    State pickup =
        sm.addState(Command.parallel(arm.horizontal(), flywheel.stop()).named("Pickup (hold)"));
    State spinUp =
        sm.addState(
            Command.parallel(arm.vertical(), flywheel.runFast())
                .until(flywheel::isAtTarget)
                .named("SpinUp until at speed"));
    State ready =
        sm.addState(
            Command.parallel(arm.vertical(), flywheel.runFast()).named("ReadyToShoot (hold)"));

    // 3. Every machine needs a starting state. Forget this and the build fails.
    sm.setInitialState(stowed);

    // 4. Wire the transitions. Each condition is checked every loop while its state is active,
    //    and fires the moment it flips from false to true.
    stowed.switchTo(pickup).when(driver.leftTrigger()); // driver asks to intake
    pickup.switchTo(stowed).when(driver.leftTrigger().negate()); // trigger released, pack up

    stowed.switchTo(spinUp).when(driver.rightTrigger()); // driver asks to shoot

    // SpinUp's command ends on its own, so this uses whenComplete(): it fires once, when the
    // command finishes. Ready runs the same pose as SpinUp. It exists so drivers, LEDs and
    // autos can tell "spinning up" apart from "ready to shoot".
    spinUp.switchTo(ready).whenComplete();

    // Releasing the right trigger backs out of either shooting state.
    sm.switchFromAny(spinUp, ready).to(stowed).when(driver.rightTrigger().negate());

    // B returns to stowed from every state.
    // switchFromAny() with no args covers every state added so far, so declare it last.
    sm.switchFromAny().to(stowed).when(driver.b());

    // onEnter/onExit run small extras on the way in and out of a state, without touching the
    // state's command. These two write markers into the log, so you can see exactly when the
    // machine entered and left ReadyToShoot.
    ready.onEnter(() -> DataLogManager.log("Superstructure: entered ReadyToShoot"));
    ready.onExit(() -> DataLogManager.log("Superstructure: left ReadyToShoot"));

    // Three more forms. Uncomment and adapt:
    //
    // whenCompleteAnd is whenComplete plus an extra check. It wins over plain whenComplete().
    //   spinUp.switchTo(stowed).whenCompleteAnd(() -> !hasGamePiece()); // lost the piece, bail
    //
    // The target state can be picked at switch time (a Supplier<State> instead of a State):
    //   spinUp.switchTo(() -> hasGamePiece() ? ready : stowed).whenComplete();
    //
    // A transition can also end the whole machine instead of moving to another state:
    //   sm.switchFromAny().toExitStateMachine().when(driver.back());

    machine = sm; // a StateMachine is a Command, so schedule it like any other
  }

  @Override
  public void start() {
    Scheduler.getDefault().schedule(machine);
  }

  @Override
  public void end() {
    Scheduler.getDefault().cancel(machine);
  }
}
