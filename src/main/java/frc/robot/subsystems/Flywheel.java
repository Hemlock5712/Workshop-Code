// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheel extends SubsystemBase {
  /** Creates a new Flywheel. */
  // Create a new CANBus with name canivore
  private final CANBus canivore = new CANBus("canivore");
  // Create the leader and follower TalonFX motors
  private final TalonFX leader = new TalonFX(21, canivore);

  private final TalonFX follower = new TalonFX(22, canivore);

  // Voltage output control for the flywheel
  private final VoltageOut voltageOut = new VoltageOut(0);

  public Flywheel() {
    // Set the follower to follow the leader motor
    follower.setControl(new Follower(leader.getDeviceID(), true));
    // Create and apply the configuration for the leader motor
    TalonFXConfiguration config = new TalonFXConfiguration();
    // Put's the motor in Coast mode to make it easier to move by hand
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    // Configure the motor to make sure positive voltage is counter clockwise
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
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
   * Sets the voltage for the flywheel.
   *
   * @param voltage The voltage to set.
   */
  public void setVoltage(double voltage) {
    // Apply the voltage output to the leader motor
    leader.setControl(voltageOut.withOutput(voltage));
  }

  // Stop the flywheel motors
  public void stop() {
    leader.stopMotor();
  }
}
