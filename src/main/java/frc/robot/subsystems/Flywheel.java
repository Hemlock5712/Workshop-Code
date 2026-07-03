// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static org.wpilib.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.utils.TalonFXUtil;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.units.measure.AngularVelocity;

/**
 * The flywheel. Two TalonFX motors: a leader (CAN 21) and a follower (CAN 22) that spins the
 * opposite direction.
 *
 * <p>The motor uses Motion Magic velocity control ({@link MotionMagicVelocityVoltage}), added in
 * 4-MotionMagic: the speed ramps up to the target instead of jumping straight to it.
 *
 * <p>New in this lesson (5-GettersAndSetters): the <b>read side</b>. {@link #getVelocity} tells you
 * how fast the wheel is really spinning, {@link #getTargetVelocity} tells you the speed it is
 * aiming for, and {@link #isAtTarget} tells you whether it is up to speed.
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

  // How close the measured speed needs to be to count as "at target".
  private static final double VELOCITY_TOLERANCE_RPS = 0.5;

  private final CANBus canivore = new CANBus("canivore");
  private final TalonFX leader = new TalonFX(21, canivore);
  private final TalonFX follower = new TalonFX(22, canivore);

  // Asks the motor to ramp to a target speed instead of jumping to it.
  private final MotionMagicVelocityVoltage velocityOut = new MotionMagicVelocityVoltage(0);

  private final AngularVelocity tolerance = RotationsPerSecond.of(VELOCITY_TOLERANCE_RPS);

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

    TalonFXUtil.applyConfigWithRetries(leader, config);
  }

  // Same setup as Arm: runRepeatedly runs the action every loop while the command is scheduled.
  // These commands are all HOLDS. A hold never finishes, so never make a sequence wait on one.
  // Need an ending? Add it where you use the command:
  // flywheel.runFast().until(flywheel::isAtTarget). The full rule is in Arm.java.

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

  /** True when the flywheel is within tolerance of its target speed. */
  public boolean isAtTarget() {
    return getVelocity().isNear(getTargetVelocity(), tolerance);
  }

  /** Current measured flywheel speed. */
  public AngularVelocity getVelocity() {
    return leader.getVelocity().getValue();
  }

  /** Speed the flywheel is currently driving toward. */
  public AngularVelocity getTargetVelocity() {
    return velocityOut.getVelocityMeasure();
  }

  private void setVelocity(double rps) {
    leader.setControl(velocityOut.withVelocity(RotationsPerSecond.of(rps)));
  }
}
