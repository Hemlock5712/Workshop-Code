// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.DriveMechanism;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Vision;
import frc.robot.utils.SimStartup;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.button.CommandNiDsXboxController;
import org.wpilib.command3.button.RobotModeTriggers;
import org.wpilib.framework.OpModeRobot;

/**
 * The main robot class. The robot's mechanisms live here as public fields. Every OpMode gets a
 * {@link Robot} in its constructor and reaches the mechanisms through it.
 *
 * <p>The framework finds the {@code @Teleop} and {@code @Autonomous} classes in {@code
 * frc.robot.opmodes} on its own and handles every mode switch, so this class has no per-mode
 * methods. It owns the drivetrain and the driver's controller, sets the joystick drive as the
 * drivetrain's default command in every mode, and runs the command scheduler every loop.
 */
public class Robot extends OpModeRobot {
  public final DriveMechanism drivetrain = new DriveMechanism();
  // Every camera on the robot, by its NetworkTables name. A second camera is a second string.
  public final Vision vision = new Vision(drivetrain, "limelight");
  // Below the drivetrain: a field cannot read one declared after it.
  public final Flywheel flywheel = new Flywheel(drivetrain);
  public final CommandNiDsXboxController driver = new CommandNiDsXboxController(0);

  private final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // top speed
  private final double maxAngularRate =
      RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 turn per second

  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(maxSpeed * 0.1)
          .withRotationalDeadband(maxAngularRate * 0.1) // ignore the sticks' bottom 10%
          .withDriveRequestType(DriveRequestType.Velocity); // closed loop, uses the drive gains

  public Robot() {
    // Joystick drive, in every mode. Set here instead of in an OpMode so it always exists: when
    // any other drivetrain command ends or is interrupted, the drivetrain goes back to the sticks.
    // In WPILib, X points forward and Y points left. The sticks read the other way around, so
    // each axis below gets a minus sign.
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(-driver.getLeftY() * maxSpeed) // left stick up = forward
                    .withVelocityY(-driver.getLeftX() * maxSpeed) // left stick left = left
                    .withRotationalRate(
                        -driver.getRightX() * maxAngularRate))); // right stick left = turn left

    // Brake while disabled, in every mode. Made here for the same reason: OpMode bindings go
    // away on a mode switch; this one never does.
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
