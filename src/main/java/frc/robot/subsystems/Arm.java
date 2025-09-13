// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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

  // Voltage output control for the arm
  private final VoltageOut voltageOut = new VoltageOut(0);

  public Arm() {
    // Create and apply the configuration for the leader motor
    TalonFXConfiguration config = new TalonFXConfiguration();
    // Put's the motor in Coast mode to make it easier to move by hand
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    // Configure the motor to make sure positive voltage is counter clockwise
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
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
   * Sets the voltage for the arm.
   *
   * @param voltage The voltage to set.
   */
  public void setVoltage(double voltage) {
    // Apply the voltage output to the leader motor
    leader.setControl(voltageOut.withOutput(voltage));
  }

  /**
   * Command to run the arm at a slow speed.
   *
   * @return The command to run the arm slowly.
   */
  public Command runSlow() {
    // Command to run the arm at a slow speed and stop it afterward
    return startEnd(() -> setVoltage(3), () -> stop());
  }

  /**
   * Command to run the arm at a fast speed.
   *
   * @return The command to run the arm quickly.
   */
  public Command runFast() {
    // Command to run the arm at a fast speed and stop it afterward
    return startEnd(() -> setVoltage(6), () -> stop());
  }

  // Stop the arm motor
  public void stop() {
    leader.stopMotor();
  }
}
