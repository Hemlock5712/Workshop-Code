// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import org.wpilib.framework.RobotBase;
import org.wpilib.hardware.hal.OpModeOption;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.simulation.DriverStationSim;

/**
 * Runs the real robot program in simulation from a test, and plays the driver station: it picks an
 * OpMode and enables, the way a person does in the Simulate Robot Code window. Tests only. Nothing
 * here ships to the robot.
 */
public final class SimRobot {
  private static volatile Robot robot;

  private SimRobot() {}

  /**
   * Starts the robot program, once for the whole test run, and waits until its OpModes are listed.
   * Later calls hand back the same robot.
   */
  public static synchronized Robot start() throws InterruptedException {
    if (robot == null) {
      Thread program =
          new Thread(
              () ->
                  RobotBase.startRobot(
                      () -> {
                        robot = new Robot();
                        return robot;
                      }),
              "robot program");
      program.setDaemon(true); // do not keep the test run alive once the tests are done
      program.start();
      waitUntil(10.0, () -> robot != null && DriverStationSim.getOpModeOptions().length > 0);
    }
    return robot;
  }

  /**
   * Picks an OpMode and enables the robot.
   *
   * @param mode the kind of OpMode, such as {@code RobotMode.AUTONOMOUS}
   * @param opModeName the name from its annotation, such as "Teleop"
   */
  public static void enable(RobotMode mode, String opModeName) throws InterruptedException {
    OpModeOption chosen = null;
    for (OpModeOption option : DriverStationSim.getOpModeOptions()) {
      if (option.getMode() == mode && option.name.equals(opModeName)) {
        chosen = option;
      }
    }
    if (chosen == null) {
      throw new IllegalArgumentException("No " + mode + " OpMode named \"" + opModeName + "\"");
    }

    // Both setRobotMode and setOpMode matter. Skip either one and the framework cannot match the
    // OpMode, so the robot stays disabled.
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setRobotMode(mode);
    DriverStationSim.setOpMode(chosen.id);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    waitUntil(5.0, () -> RobotBase.isEnabled());
  }

  /** Disables the robot, which also ends the running OpMode. */
  public static void disable() throws InterruptedException {
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    waitUntil(5.0, () -> RobotBase.isDisabled());
  }

  /** Lets the robot program run for this long, in real time. */
  public static void runFor(double seconds) throws InterruptedException {
    Thread.sleep((long) (seconds * 1000));
  }

  /** Checks a condition every loop until it is true, and fails the test when time runs out. */
  public static void waitUntil(double timeoutSeconds, Condition condition)
      throws InterruptedException {
    long end = System.nanoTime() + (long) (timeoutSeconds * 1e9);
    while (!condition.isTrue()) {
      if (System.nanoTime() > end) {
        throw new AssertionError("Still waiting after " + timeoutSeconds + " seconds");
      }
      Thread.sleep(20);
    }
  }

  /** Something that is either true or not yet. */
  public interface Condition {
    boolean isTrue();
  }
}
