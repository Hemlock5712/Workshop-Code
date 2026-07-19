// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.opmodes;

import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.Robot;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.DriveMechanism;
import org.wpilib.command3.button.CommandNiDsXboxController;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.opmode.Teleop;

/**
 * The driver's controls. The framework builds this class when "Teleop" is picked on the driver
 * station. The button bindings made in the constructor belong to this OpMode, and the framework
 * removes them on a mode switch. No cleanup code needed.
 *
 * <p>The joystick-drive default command lives here, not in {@link Robot}, because it needs this
 * OpMode's controller. Add a second {@code @Teleop} class and it shows up as another choice on the
 * driver station.
 */
@Teleop(name = "Teleop")
public class TeleopOpMode extends PeriodicOpMode {
  private final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // top speed
  private final double maxAngularRate =
      RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 turn per second

  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(maxSpeed * 0.1)
          .withRotationalDeadband(maxAngularRate * 0.1) // ignore the sticks' bottom 10%
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // plain voltage, no wheel PID

  private final CommandNiDsXboxController driver = new CommandNiDsXboxController(0);

  public TeleopOpMode(Robot robot) {
    final DriveMechanism drivetrain = robot.drivetrain;

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

    // Left bumper: make the robot's current facing the new "forward".
    driver.leftBumper().onTrue(drivetrain.seedFieldCentric());
  }
}
