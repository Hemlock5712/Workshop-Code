// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.subsystems.DriveMechanism;
import frc.robot.utils.SimStartup;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.button.RobotModeTriggers;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.framework.OpModeRobot;
import org.wpilib.system.DataLogManager;

/**
 * The main robot class. The robot's mechanisms live here as public fields. Every OpMode gets a
 * {@link Robot} in its constructor and reaches the mechanisms through it.
 *
 * <p>The framework finds the {@code @Teleop} and {@code @Autonomous} classes in {@code
 * frc.robot.opmodes} on its own and handles every mode switch, so this class has no per-mode
 * methods. It owns the drivetrain and runs the command scheduler every loop.
 */
public class Robot extends OpModeRobot {
  public final DriveMechanism drivetrain = new DriveMechanism();

  public Robot() {
    // Start logging. DataLogManager records every NetworkTables value change (including
    // everything Telemetry publishes) into a .wpilog file, plus console output. startDataLog
    // also records the driver station state and joystick data. Logs go to ./logs in simulation
    // and to a USB drive (or /home/systemcore/logs) on the real robot.
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

    // Brake while disabled, in every mode. This binding is made here instead of in an OpMode so
    // it always exists. OpMode bindings go away on a mode switch; this one never does.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(drivetrain.applyRequest(() -> idle));
  }

  @Override
  public void simulationInit() {
    // Lets simulation start enabled when a launcher asks for it. Does nothing in a normal run.
    // See SimStartup for details.
    SimStartup.arm();
  }

  @Override
  public void robotPeriodic() {
    Scheduler.getDefault().run();
  }
}
