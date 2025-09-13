// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheel extends SubsystemBase {
  /** Creates a new Flywheel. */
  // Create a new CANBus with name canivore
  private final CANBus canivore = new CANBus("canivore");
  // Create the leader and follower TalonFX motors
  private final TalonFX leader = new TalonFX(21, canivore);

  private final TalonFX follower = new TalonFX(22, canivore);

  // Velocity output control for the flywheel
  private final VelocityVoltage velocityOut = new VelocityVoltage(0);

  public Flywheel() {
    // Set the follower to follow the leader motor
    follower.setControl(new Follower(leader.getDeviceID(), true));
    // Create and apply the configuration for the leader motor
    TalonFXConfiguration config = new TalonFXConfiguration();
    // Put's the motor in Coast mode to make it easier to move by hand
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    // Configure the motor to make sure positive voltage is counter clockwise
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.Slot0.kS = 0.0; // Static gain
    config.Slot0.kV = 0.0; // Velocity gain
    config.Slot0.kP = 0.0; // Proportional gain
    // Try to apply config multiple time. Break after successfully applying
    for (int i = 0; i < 2; ++i) {
      var status = leader.getConfigurator().apply(config);
      if (status.isOK()) break;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Sets the velocity for the flywheel.
   *
   * @param velocity The velocity to set.
   */
  public void setVelocity(AngularVelocity velocity) {
    // Apply the velocity output to the leader motor
    leader.setControl(velocityOut.withVelocity(velocity));
  }

  /**
   * Command to run the flywheel at a slow speed.
   *
   * @return The command to run the flywheel slowly.
   */
  public Command runSlow() {
    // Command to run the flywheel at a slow speed
    return runOnce(() -> setVelocity(RotationsPerSecond.of(0.25)));
  }

  /**
   * Command to run the flywheel at a fast speed.
   *
   * @return The command to run the flywheel fast.
   */
  public Command runFast() {
    // Command to run the flywheel at a fast speed
    return runOnce(() -> setVelocity(DegreesPerSecond.of(360)));
  }

  // Stop the flywheel motors
  public void stop() {
    leader.stopMotor();
  }
}
