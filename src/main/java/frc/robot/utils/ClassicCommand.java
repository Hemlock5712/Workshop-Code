// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.Arrays;
import java.util.Set;
import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Mechanism;

/**
 * The classic four-method command style, rebuilt on top of Commands v3.
 *
 * <p>Commands v3 usually writes a command as one straight-line body (see {@code
 * mechanism.run(...)}). But for a command with clear separate steps - or if you learned the older
 * style - the four methods below can be easier to follow. Extend this class and override what you
 * need. The result is a normal {@link Command}: schedule it or bind it to a button like any other.
 *
 * <p>The lifecycle:
 *
 * <ul>
 *   <li>{@link #initialize()} runs once when the command starts.
 *   <li>{@link #execute()} runs every loop while the command is active.
 *   <li>{@link #isFinished()} is checked every loop, right after {@code execute}. Return true to
 *       finish.
 *   <li>{@link #end(boolean)} runs once when the command ends. {@code interrupted} is true when
 *       another command took one of this command's mechanisms away.
 * </ul>
 *
 * <p>Under the hood it is just a loop that calls these methods - see {@link #run} at the bottom of
 * this file.
 *
 * <p>Example:
 *
 * <pre>{@code
 * public class DriveDistance extends ClassicCommand {
 *   private final Drive drive;
 *   private final double meters;
 *
 *   public DriveDistance(Drive drive, double meters) {
 *     super("DriveDistance", drive); // command name + required mechanisms
 *     this.drive = drive;
 *     this.meters = meters;
 *   }
 *
 *   @Override protected void initialize()       { drive.resetEncoders(); }
 *   @Override protected void execute()          { drive.arcade(0.5, 0); }
 *   @Override protected boolean isFinished()    { return drive.distance() >= meters; }
 *   @Override protected void end(boolean intr)  { drive.stop(); }
 * }
 * }</pre>
 */
public abstract class ClassicCommand implements Command {
  private final String name;
  private final Set<Mechanism> requirements;

  /**
   * Creates a classic-style command.
   *
   * @param name The command name (shows up in telemetry).
   * @param requirements The mechanisms this command owns while it runs.
   */
  protected ClassicCommand(String name, Mechanism... requirements) {
    this.name = name;
    this.requirements = Set.copyOf(Arrays.asList(requirements));
  }

  /** Runs once when the command starts. Override to set up state. */
  protected void initialize() {}

  /** Runs every loop while the command is active. Override to do the work. */
  protected void execute() {}

  /**
   * Checked every loop, right after {@link #execute()}.
   *
   * @return true to finish the command, false to keep running.
   */
  protected boolean isFinished() {
    return false;
  }

  /**
   * Runs once when the command ends. Keep this to single-shot cleanup (for example, stopping a
   * motor); don't loop here.
   *
   * @param interrupted false if {@link #isFinished()} ended the command, true if it was interrupted
   *     by another command claiming one of its mechanisms.
   */
  protected void end(boolean interrupted) {}

  @Override
  public final void run(Coroutine coroutine) {
    initialize();
    while (true) {
      execute();
      if (isFinished()) {
        break;
      }
      coroutine.yield();
    }
    end(false); // natural finish
  }

  @Override
  public final void onCancel() {
    // The scheduler drops the command's loop when it is interrupted, so this is the only cleanup
    // hook that runs.
    end(true);
  }

  @Override
  public final String name() {
    return name;
  }

  @Override
  public final Set<Mechanism> requirements() {
    return requirements;
  }
}
