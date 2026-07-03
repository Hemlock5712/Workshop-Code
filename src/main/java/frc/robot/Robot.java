// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Flywheel;
import frc.robot.utils.SimStartup;
import org.wpilib.command3.Scheduler;
import org.wpilib.framework.OpModeRobot;

/**
 * The main robot class. The robot's mechanisms live here as public fields. Every OpMode gets a
 * {@link Robot} in its constructor and reaches the mechanisms through it.
 *
 * <p>The arm and flywheel commands use closed-loop control. This class stays the same in every
 * lesson: it owns the mechanisms and runs the scheduler every loop.
 */
public class Robot extends OpModeRobot {
  // The robot's mechanisms. Public so OpModes can use them.
  public final Arm arm = new Arm();
  public final Flywheel flywheel = new Flywheel();

  public Robot() {}

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
