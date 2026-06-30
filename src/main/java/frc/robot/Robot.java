// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.subsystems.DriveMechanism;
import frc.robot.utils.SimStartup;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.button.RobotModeTriggers;
import org.wpilib.framework.OpModeRobot;

/**
 * Owns the robot's shared hardware in one place. With the OpMode framework there is no {@code
 * RobotContainer}: the subsystems live here as public fields, and each OpMode in {@code
 * frc.robot.opmodes} reaches them through the {@link Robot} reference it is constructed with.
 *
 * <p>The framework auto-discovers the {@code @Teleop}/{@code @Autonomous} classes in this package
 * (and subpackages) and handles every mode transition, so this class has no per-mode init/periodic
 * methods - only the always-on scheduler tick. Selecting a different mode on the driver station
 * constructs that OpMode and tears down the previous one (its button bindings are scoped to it and
 * removed automatically).
 */
public class Robot extends OpModeRobot {
  public final DriveMechanism drivetrain = new DriveMechanism();

  public Robot() {
    // Brake while disabled, in every mode. Created here (before any OpMode is selected) so the
    // binding is global; the opmodes' bindings are scoped to their OpMode and removed on a switch.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(drivetrain.applyRequest(() -> idle));
  }

  @Override
  public void simulationInit() {
    // Headless auto-enable for agent / CI runs. No-op unless -Dfrc.sim.startMode is set (the
    // simulateJavaAgent Gradle task sets it). See SimStartup and the run-sim skill.
    SimStartup.arm();
  }

  @Override
  public void robotPeriodic() {
    Scheduler.getDefault().run();
  }
}
