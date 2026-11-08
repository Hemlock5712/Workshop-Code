// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.opmodes;

import bline.commands.FollowPath;
import frc.robot.Robot;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.system.DataLogManager;
import org.wpilib.tunable.Selectable;
import org.wpilib.tunable.Tunables;

/**
 * The robot's only autonomous OpMode. Pick it on the driver station, then pick which routine it
 * runs from the {@code Tunables/Auto} drop-down on the dashboard.
 *
 * <p>Each routine is a coroutine that drives one or more BLine paths in order. The paths are drawn
 * in the BLine editor and saved under {@code deploy/autos/paths/}.
 */
@Autonomous(name = "Auto")
public class AutoOpMode extends PeriodicOpMode {
  private final Selectable<Command> autoChooser = new Selectable<>();
  private Command routine;

  public AutoOpMode(Robot robot) {
    // Event triggers are the markers along a path. The key in quotes must match the marker's key
    // in the BLine editor exactly. This one is a stand-in that only prints a line to the driver
    // station console and the log. A robot with an intake would start the intake here instead.
    FollowPath.registerEventTrigger(
        "atMidpoint", () -> DataLogManager.log("Leave Start: reached the midpoint"));

    // Read the path file now, so a misspelled name fails when this OpMode is picked.
    // withPoseReset puts odometry at the path's first point before it drives.
    final var leaveStartPath = robot.drivetrain.followPath("leave-start").withPoseReset();

    // The routine owns no mechanism itself. The path command it awaits claims the drivetrain for
    // as long as it drives, and a second path would go on the next line.
    final Command leaveStart =
        Command.noRequirements(
                coroutine -> {
                  coroutine.await(leaveStartPath);
                })
            .named("Leave Start");
    final Command none = Command.noRequirements(coroutine -> {}).named("None");

    autoChooser.addDefault("Leave Start", leaveStart);
    autoChooser.add("None", none);
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

  /**
   * Canceling the path sends the drivetrain a zero. Clearing the triggers drops any marker the path
   * passed that has not run yet, so nothing fires after auto is over.
   */
  @Override
  public void end() {
    Scheduler.getDefault().cancel(routine);
    FollowPath.clearPendingEventTriggers();
  }

  /** Takes the drop-down off the dashboard when another OpMode is picked. */
  @Override
  public void close() {
    Tunables.remove("Auto");
  }
}
