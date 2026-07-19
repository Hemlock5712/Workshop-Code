// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import org.wpilib.framework.RobotBase;
import org.wpilib.hardware.hal.OpModeOption;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.simulation.DriverStationSim;

/**
 * Auto-enables the robot in simulation. Normally the sim starts disabled and waits for someone to
 * click "Enable". If the {@code frc.sim.startMode} system property is set, this class does the
 * clicking for you: it picks an OpMode and enables the robot, the same way a person would in the
 * sim driver station.
 *
 * <p>Call {@link #arm()} once from {@link frc.robot.Robot#simulationInit()}. It only does anything
 * in simulation, and only when the property is set.
 *
 * <p>Property values:
 *
 * <ul>
 *   <li>{@code auto} / {@code teleop} / {@code utility} - enable in that mode, using the first
 *       OpMode of that kind.
 *   <li>{@code <mode>:<OpMode name>} - for example {@code auto:Drive To Pose} - enable in that mode
 *       and pick the OpMode with that name.
 *   <li>empty / {@code disabled} / unset - do nothing. The robot stays disabled, which is the
 *       normal behavior for {@code simulateJava}.
 * </ul>
 */
public final class SimStartup {
  private SimStartup() {}

  /** Reads {@code frc.sim.startMode} and, in simulation, picks an OpMode and enables the robot. */
  public static void arm() {
    if (!RobotBase.isSimulation()) {
      return;
    }

    String spec = System.getProperty("frc.sim.startMode", "").trim();
    if (spec.isEmpty() || spec.equalsIgnoreCase("disabled")) {
      return; // Property not set. Stay disabled like a normal sim run.
    }

    // Split "<mode>" or "<mode>:<name>".
    String modeStr = spec;
    String wantName = null;
    int colon = spec.indexOf(':');
    if (colon >= 0) {
      modeStr = spec.substring(0, colon).trim();
      wantName = spec.substring(colon + 1).trim();
    }

    RobotMode mode =
        switch (modeStr.toLowerCase()) {
          case "auto", "autonomous" -> RobotMode.AUTONOMOUS;
          case "teleop", "teleoperated" -> RobotMode.TELEOPERATED;
          case "utility", "test" -> RobotMode.UTILITY;
          default -> null;
        };
    if (mode == null) {
      System.err.println(
          "[SimStartup] Unknown mode '" + spec + "' (use auto|teleop|utility); staying disabled.");
      return;
    }

    OpModeOption chosen = null;
    for (OpModeOption option : DriverStationSim.getOpModeOptions()) {
      if (option.getMode() != mode) {
        continue;
      }
      if (wantName == null || option.name.equalsIgnoreCase(wantName)) {
        chosen = option;
        break;
      }
    }
    if (chosen == null) {
      System.err.println(
          "[SimStartup] No "
              + mode
              + " OpMode"
              + (wantName != null ? " named \"" + wantName + "\"" : "")
              + " found; staying disabled.");
      return;
    }

    // Both calls below matter: setOpMode picks which OpMode, and setRobotMode sets auto/teleop.
    // Skip either one and the framework can't match the OpMode, so the robot stays disabled.
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setRobotMode(mode);
    DriverStationSim.setOpMode(chosen.id);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();

    System.out.println(
        "[SimStartup] Headless start: enabled=true mode="
            + mode
            + " opmode=\""
            + chosen.name
            + "\"");
  }
}
