// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.utils.TalonFXUtil;
import org.wpilib.command3.Mechanism;

/**
 * The arm. One TalonFX motor plus a CANcoder that measures the arm's angle.
 *
 * <p>Every mechanism in this project follows the same pattern: extend {@code Mechanism}, keep the
 * hardware in private fields, and set it up once in the constructor.
 *
 * <p>Right now the arm can only do one thing: push a voltage at the motor. The methods are public
 * so you can call them and watch the arm move. In the next lesson (2-Commands) we wrap them in
 * commands.
 */
public class Arm extends Mechanism {
  private final CANBus canivore = new CANBus("canivore");
  private final TalonFX motor = new TalonFX(31, canivore);
  private final CANcoder encoder = new CANcoder(32, canivore);

  // Pushes a set voltage at the motor. No sensors involved.
  private final VoltageOut voltageOut = new VoltageOut(0);

  public Arm() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast; // easy to move by hand
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Use the CANcoder for position, so the motor knows the arm's real angle.
    config.Feedback.withRemoteCANcoder(encoder);

    TalonFXUtil.applyConfigWithRetries(motor, config);
  }

  /**
   * Push the arm with a fixed voltage. Positive voltage moves the arm counter-clockwise.
   *
   * @param voltage The voltage to apply.
   */
  public void setVoltage(double voltage) {
    motor.setControl(voltageOut.withOutput(voltage));
  }

  /** Stop the arm motor. */
  public void stop() {
    motor.stopMotor();
  }
}
