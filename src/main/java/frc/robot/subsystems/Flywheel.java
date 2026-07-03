// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static org.wpilib.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
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
 * <p>New in this lesson: closed-loop <b>velocity</b> control ({@link VelocityVoltage}). Each
 * command picks a speed in rotations per second, and the motor's PID holds that speed even when
 * game pieces slow the wheel down.
 */
public class Flywheel extends Mechanism {
  // Shooting speeds (rotations per second).
  private static final double SLOW_SPEED_RPS = 25.0;
  private static final double FAST_SPEED_RPS = 75.0;

  // PID + feedforward gains.
  private static final double kS = 0.0; // overcomes friction
  private static final double kV = 0.125; // volts per rotation-per-second
  private static final double kP = 0.0; // correction strength

  private final CANBus canivore = new CANBus("canivore");
  private final TalonFX leader = new TalonFX(21, canivore);
  private final TalonFX follower = new TalonFX(22, canivore);

  // Asks the motor's PID to hold a target speed.
  private final VelocityVoltage velocityOut = new VelocityVoltage(0);

  public Flywheel() {
    // The follower copies the leader, spinning the opposite direction.
    follower.setControl(new Follower(leader.getDeviceID(), MotorAlignmentValue.Opposed));

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast; // easy to spin by hand
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.Slot0.kS = kS;
    config.Slot0.kV = kV;
    config.Slot0.kP = kP;

    TalonFXUtil.applyConfigWithRetries(leader, config);
  }

  // Same setup as Arm: runRepeatedly runs the action every loop while the command is scheduled.
  // These commands are all HOLDS. A hold never finishes, so never make a sequence wait on one.
  // Need an ending? Add it where you use the command: flywheel.runFast().until(someCondition).
  // The full rule is in Arm.java.

  /** Spin the flywheel at the slow speed and hold it. Never finishes. */
  public Command runSlow() {
    return runRepeatedly(() -> setVelocity(SLOW_SPEED_RPS)).named("runSlow (hold)");
  }

  /** Spin the flywheel at the fast speed and hold it. Never finishes. */
  public Command runFast() {
    return runRepeatedly(() -> setVelocity(FAST_SPEED_RPS)).named("runFast (hold)");
  }

  /** Stop the flywheel and keep it stopped. Never finishes. */
  public Command stop() {
    return runRepeatedly(leader::stopMotor).named("stop (hold)");
  }

  private void setVelocity(double rps) {
    leader.setControl(velocityOut.withVelocity(RotationsPerSecond.of(rps)));
  }
}
