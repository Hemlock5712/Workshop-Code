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
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.utils.TalonFXUtil;
import org.wpilib.command3.Mechanism;

/**
 * The flywheel. Two TalonFX motors: a leader (CAN 21) and a follower (CAN 22) that spins the
 * opposite direction.
 *
 * <p>Same pattern as {@link Arm}: extend {@code Mechanism}, keep the hardware in private fields,
 * set it up once in the constructor. For now it can only push a voltage at the motors.
 */
public class Flywheel extends Mechanism {
  private final CANBus canivore = new CANBus("canivore");
  private final TalonFX leader = new TalonFX(21, canivore);
  private final TalonFX follower = new TalonFX(22, canivore);

  // Pushes a set voltage at the leader motor. No sensors involved.
  private final VoltageOut voltageOut = new VoltageOut(0);

  public Flywheel() {
    // The follower copies the leader, spinning the opposite direction.
    follower.setControl(new Follower(leader.getDeviceID(), MotorAlignmentValue.Opposed));

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast; // easy to spin by hand
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    TalonFXUtil.applyConfigWithRetries(leader, config);
  }

  /**
   * Spin the flywheel with a fixed voltage.
   *
   * @param voltage The voltage to apply.
   */
  public void setVoltage(double voltage) {
    leader.setControl(voltageOut.withOutput(voltage));
  }

  /** Stop the flywheel motors. */
  public void stop() {
    leader.stopMotor();
  }
}
