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
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;

/**
 * The flywheel. Two TalonFX motors: a leader (CAN 21) and a follower (CAN 22) that spins the
 * opposite direction.
 *
 * <p>Same idea as {@link Arm}: the raw setter is now private, and the flywheel offers commands
 * instead. The commands still push plain voltage. In the next lesson (3-PID) we switch to real
 * velocity control.
 */
public class Flywheel extends Mechanism {
  // Voltages for the two example commands.
  private static final double SLOW_VOLTAGE = 3.0;
  private static final double FAST_VOLTAGE = 6.0;

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

  // Same setup as Arm: runRepeatedly runs the action every loop while the command is scheduled.
  // These commands are all HOLDS. A hold never finishes, so never make a sequence wait on one.
  // Need an ending? Add it where you use the command: flywheel.runFast().until(someCondition).
  // The full rule is in Arm.java.

  /** Spin the flywheel with a gentle voltage and hold it there. Never finishes. */
  public Command runSlow() {
    return runRepeatedly(() -> setVoltage(SLOW_VOLTAGE)).named("runSlow (hold)");
  }

  /** Spin the flywheel with a stronger voltage and hold it there. Never finishes. */
  public Command runFast() {
    return runRepeatedly(() -> setVoltage(FAST_VOLTAGE)).named("runFast (hold)");
  }

  /** Stop the flywheel and keep it stopped. Never finishes. */
  public Command stop() {
    return runRepeatedly(leader::stopMotor).named("stop (hold)");
  }

  private void setVoltage(double voltage) {
    leader.setControl(voltageOut.withOutput(voltage));
  }
}
