// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.mechanisms;

import static org.wpilib.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;

/**
 * The flywheel. Two TalonFX motors: a leader (CAN 21) and a follower (CAN 22) that spins the
 * opposite direction.
 *
 * <p>New in this lesson: <b>Motion Magic</b> velocity control ({@link MotionMagicVelocityVoltage}).
 * Same gains as mech-3-PID, but the speed now ramps up to the target along an acceleration limit instead
 * of jumping straight to it.
 */
public class Flywheel extends Mechanism {
  // Shooting speeds (rotations per second).
  private static final double SLOW_SPEED_RPS = 25.0;
  private static final double FAST_SPEED_RPS = 75.0;

  // PID + feedforward gains.
  private static final double kS = 0.0; // overcomes friction
  private static final double kV = 0.125; // volts per rotation-per-second
  private static final double kP = 0.0; // correction strength

  // Motion Magic limits: how fast the wheel may spin and how quickly it may speed up.
  private static final double MOTION_MAGIC_CRUISE_VELOCITY = 100.0; // top speed (rot/s)
  private static final double MOTION_MAGIC_ACCELERATION = 1000.0; // ramp rate (rot/s²)

  private final CANBus canivore = new CANBus("canivore");
  private final TalonFX leader = new TalonFX(21, canivore);
  private final TalonFX follower = new TalonFX(22, canivore);

  // Asks the motor to ramp to a target speed instead of jumping to it.
  private final MotionMagicVelocityVoltage velocityOut = new MotionMagicVelocityVoltage(0);

  public Flywheel() {
    // The follower copies the leader, spinning the opposite direction.
    follower.setControl(new Follower(leader.getDeviceID(), MotorAlignmentValue.Opposed));

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast; // easy to spin by hand
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.Slot0.kS = kS;
    config.Slot0.kV = kV;
    config.Slot0.kP = kP;
    config.MotionMagic.MotionMagicCruiseVelocity = MOTION_MAGIC_CRUISE_VELOCITY;
    config.MotionMagic.MotionMagicAcceleration = MOTION_MAGIC_ACCELERATION;

    leader.getConfigurator().apply(config);
  }

  // Same setup as Arm. runRepeatedly runs the action every loop while the command is scheduled.
  // These commands are all holds. A hold never finishes, so never make a sequence wait on one.
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
