// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.opmodes;

import static org.wpilib.units.Units.Seconds;

import com.pathplanner.lib.command3.AutoBuilder;
import com.pathplanner.lib.command3.NamedCommands;
import frc.robot.Robot;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.tunable.Selectable;
import org.wpilib.tunable.Tunables;

/**
 * The robot's only autonomous OpMode. Pick it on the driver station, then pick which PathPlanner
 * auto it runs from the {@code Tunables/Auto} drop-down on the dashboard.
 *
 * <p>Each auto puts the robot at its first path's starting pose, then drives each path and runs
 * each named command in the order the PathPlanner app shows them. All of that is in the .auto file.
 */
@Autonomous(name = "Auto")
public class AutoOpMode extends PeriodicOpMode {
  private final Selectable<Command> autoChooser;
  private Command routine;

  public AutoOpMode(Robot robot) {
    // Named commands are the boxes an auto runs between paths, or the event markers along a path.
    // The name in quotes must match the name in the PathPlanner app exactly. A named command can
    // use any mechanism on robot, or several at once.
    // These two are stand-ins that only take time. A robot with a shooter and an intake would
    // register the real commands here instead.
    NamedCommands.registerCommand(
        "Shoot",
        Command.noRequirements(coroutine -> coroutine.wait(Seconds.of(1.0))).named("Shoot"));
    NamedCommands.registerCommand(
        "Intake", Command.noRequirements(coroutine -> coroutine.park()).named("Intake"));

    // PathPlanner fills this with one choice per auto drawn in the app, plus "None". It loads
    // every auto, so the named commands above have to be registered first.
    autoChooser = AutoBuilder.buildAutoChooser("Leave Start");
    Tunables.publish("Auto", autoChooser);
  }

  /**
   * Reads the drop-down here, not in the constructor. The constructor runs when the OpMode is
   * picked on the driver station, and the drop-down can still change after that.
   */
  @Override
  public void start() {
    routine = autoChooser.getSelected();
    Scheduler.getDefault().schedule(routine);
  }

  @Override
  public void end() {
    Scheduler.getDefault().cancel(routine);
  }

  /** Takes the drop-down off the dashboard when another OpMode is picked. */
  @Override
  public void close() {
    Tunables.remove("Auto");
  }
}
