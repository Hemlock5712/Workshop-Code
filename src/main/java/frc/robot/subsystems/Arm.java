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
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;

/**
 * The arm. One TalonFX motor plus a CANcoder that measures the arm's angle.
 *
 * <p>New in this lesson: the raw setter is now {@code private}, and the arm offers <b>commands</b>
 * instead (each method returns a {@link Command}). Anything that wants to move the arm goes through
 * a command. That is how the scheduler keeps two things from fighting over the motor.
 *
 * <p>The commands still just push a voltage, so where the arm ends up depends on gravity and
 * friction. In the next lesson (3-PID) we make the motor aim for a real target instead.
 */
public class Arm extends Mechanism {
  // Voltages for the two example commands.
  private static final double SLOW_VOLTAGE = 3.0;
  private static final double FAST_VOLTAGE = 6.0;

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

  // Each command below uses runRepeatedly, which runs its action every loop while the command is
  // scheduled. Re-sending the request every loop also restores it if a motor controller reboots.
  //
  // THE ONE RULE: these commands are all HOLDS. A hold never finishes, so never make anything
  // WAIT for a hold. A hold inside Command.sequence sticks there forever. If a step needs an
  // ending, add one where you use the command instead of writing a new method here:
  //
  //   arm.runSlow().until(someCondition)   // ends when the condition turns true
  //
  // Every hold has "(hold)" in its name. Names show up on the dashboard and in logs, so if a
  // stuck sequence is sitting on a "(hold)", you found the bug.

  /** Push the arm with a gentle voltage and keep pushing. Never finishes. See the rule above. */
  public Command runSlow() {
    return runRepeatedly(() -> setVoltage(SLOW_VOLTAGE)).named("runSlow (hold)");
  }

  /** Push the arm with a stronger voltage and keep pushing. Never finishes. */
  public Command runFast() {
    return runRepeatedly(() -> setVoltage(FAST_VOLTAGE)).named("runFast (hold)");
  }

  /** Stop the arm motor and keep it stopped. Never finishes. */
  public Command stop() {
    return runRepeatedly(motor::stopMotor).named("stop (hold)");
  }

  private void setVoltage(double voltage) {
    motor.setControl(voltageOut.withOutput(voltage));
  }
}
