// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import org.wpilib.framework.RobotBase;
import org.wpilib.hardware.hal.OpModeOption;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.simulation.DriverStationSim;

/**
 * Headless simulation auto-enable. Lets an agent / CI run the robot in simulation and have it
 * actually start playing, instead of sitting on the disabled screen waiting for a human to click
 * "Enable" in the sim GUI.
 *
 * <p>Call {@link #arm()} once from {@link frc.robot.Robot#simulationInit()}. In simulation only, it
 * reads the system property {@code frc.sim.startMode} (set from the Gradle command line - see
 * {@code build.gradle} and the {@code run-sim} skill) and drives {@link DriverStationSim} to select
 * an OpMode and enable the robot.
 *
 * <p>Property values:
 *
 * <ul>
 *   <li>{@code auto} / {@code teleop} / {@code utility} - enable in that mode, picking the first
 *       OpMode of that kind that the framework discovered.
 *   <li>{@code <mode>:<OpMode name>} - e.g. {@code auto:Drive To Pose} - enable in that mode and
 *       pick the OpMode whose {@code @Autonomous/@Teleop/@Utility} {@code name} matches.
 *   <li>empty / {@code disabled} / unset - do nothing (robot stays disabled). This is the default
 *       for a plain {@code simulateJava}.
 * </ul>
 *
 * <p>How it works: the OpMode framework registers every {@code @Autonomous/@Teleop/@Utility} class
 * with the driver station during the {@code OpModeRobot} constructor, so by the time {@code
 * simulationInit()} runs the options are published and {@link DriverStationSim#getOpModeOptions()}
 * can resolve a name to an opmode id. Selecting an opmode id + enabling is exactly what a human
 * does in the sim DS; the first robot loop then constructs that OpMode and calls {@code start()}.
 */
public final class SimStartup {
  private SimStartup() {}

  /** Reads {@code frc.sim.startMode} and, in simulation, selects an OpMode and enables the DS. */
  public static void arm() {
    if (!RobotBase.isSimulation()) {
      return;
    }

    String spec = System.getProperty("frc.sim.startMode", "").trim();
    if (spec.isEmpty() || spec.equalsIgnoreCase("disabled")) {
      return; // Stay disabled - normal interactive sim behavior.
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

    // The control word is assembled from separate sim fields: setOpMode supplies the name-hash
    // portion of the id, and setRobotMode supplies the mode bits. Both are required - without
    // setRobotMode the id the framework reads back is missing its mode bits and won't match the
    // opmode it registered ("No OpMode found for mode ...").
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
