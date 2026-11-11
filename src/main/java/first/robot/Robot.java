// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import first.robot.mechanisms.Arm;
import first.robot.mechanisms.Flywheel;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.button.CommandNiDsXboxController;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.framework.OpModeRobot;
import org.wpilib.system.DataLogManager;
import org.wpilib.telemetry.Telemetry;

/**
 * The main robot class. The robot's mechanisms and the driver's controller live here as public
 * fields. Every OpMode gets a {@link Robot} in its constructor and reaches them through it.
 *
 * <p>It owns the mechanisms, runs the scheduler every loop, and logs every mechanism every loop.
 */
public class Robot extends OpModeRobot {
  // The robot's mechanisms. Public so OpModes can use them.
  public final Arm arm = new Arm();
  public final Flywheel flywheel = new Flywheel();

  // The driver's controller on port 0. OpModes bind its buttons through robot.driver.
  public final CommandNiDsXboxController driver = new CommandNiDsXboxController(0);

  public Robot() {
    // Record every logged value, plus the driver station, to a .wpilog file.
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
  }

  @Override
  public void robotPeriodic() {
    Scheduler.getDefault().run();
    Telemetry.log("Arm", arm);
    Telemetry.log("Flywheel", flywheel);
  }
}
