// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  // Create a new CANBus with name canivore
  private final CANBus canivore = new CANBus("canivore");
  // Create the leader TalonFX motor and a CANcoder for position feedback
  private final TalonFX leader = new TalonFX(31, canivore);
  // Create and absolute encoder that the motor can refrence for position
  private final CANcoder encoder = new CANcoder(32, canivore);

  // Position output control for the arm
  private final MotionMagicVoltage positionOut = new MotionMagicVoltage(0);

  // Tolerance for the arm position
  private final Angle tolerance = Degrees.of(1.0);

  public Arm() {
    // Create and apply the configuration for the leader motor
    TalonFXConfiguration config = new TalonFXConfiguration();
    // Put's the motor in Coast mode to make it easier to move by hand
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    // Configure the motor to make sure positive voltage is counter clockwise
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine; // Use cosine gravity compensation
    config.Slot0.kG = 0.0; // Gravity gain
    config.Slot0.kS = 0.0; // Static gain
    config.Slot0.kP = 0.0; // Proportional gain
    config.Slot0.kD = 0.0; // Derivative gain
    config.MotionMagic.MotionMagicCruiseVelocity = 0.0; // Max velocity
    config.MotionMagic.MotionMagicAcceleration = 0.0; // Max acceleration allowed
    // Configure the leader motor to use the CANcoder for position feedback
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    config.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
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
   * Sets the position for the arm.
   *
   * @param position The position to set.
   */
  public void setPosition(Angle position) {
    // Apply the position output to the leader motor
    leader.setControl(positionOut.withPosition(position));
  }

  /**
   * Command to run the arm to vertical position.
   *
   * @return The command to run the arm vertical.
   */
  public Command vertical() {
    // Command to run the arm to vertical position and stop it afterward
    return startEnd(() -> setPosition(Degrees.of(90)), () -> stop());
  }

  /**
   * Command to run the arm to a horizontal position.
   *
   * @return The command to run the arm horizontal.
   */
  public Command horizontal() {
    // Command to run the arm to horizontal position and stop it afterward
    return startEnd(() -> setPosition(Rotations.of(0.5)), () -> stop());
  }

  /**
   * Checks if the arm is at its target position.
   *
   * @return true if at target position, false otherwise
   */
  public boolean isAtTarget() {
    // Check if the arm's position is near the target position
    return getPosition().isNear(getTargetPosition(), tolerance);
  }

  /**
   * Gets the current position of the arm.
   *
   * @return The current position of the arm.
   */
  public Angle getPosition() {
    // Get the current position of the arm from the CANcoder
    return encoder.getPosition().getValue();
  }

  /**
   * Gets the target position for the arm.
   *
   * @return The target position of the arm.
   */
  public Angle getTargetPosition() {
    // Return the target position
    return positionOut.getPositionMeasure();
  }

  // Stop the arm motor
  public void stop() {
    leader.stopMotor();
  }
}
