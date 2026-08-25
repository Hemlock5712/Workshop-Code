// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.mechanisms;

import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.RotationsPerSecondPerSecond;
import static org.wpilib.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.units.measure.AngularVelocity;

/**
 * The flywheel. One TalonFX motor (CAN 21). No CANcoder: the encoder inside the motor already
 * measures speed.
 *
 * <p>The motor uses Motion Magic velocity control ({@link MotionMagicVelocityVoltage}), added one
 * lesson back. The speed ramps up to the target instead of jumping straight to it.
 *
 * <p>New in this lesson (mech-4-ReadingState): the <b>read side</b>. {@link #getVelocity} tells you
 * how fast the wheel is really spinning, {@link #getTargetVelocity} tells you the speed it is
 * aiming for, and {@link #isAtTarget} tells you whether it is up to speed.
 */
public class Flywheel extends Mechanism {
  private final CANBus canivore = new CANBus("canivore");
  private final TalonFX motor = new TalonFX(21, canivore);

  // Asks the motor to ramp to a target speed instead of jumping to it.
  private final MotionMagicVelocityVoltage velocityOut = new MotionMagicVelocityVoltage(0);

  // How close the measured speed needs to be to count as "at target".
  private final AngularVelocity tolerance = RotationsPerSecond.of(0.5);

  public Flywheel() {
    // Pasted from Tuner X. Every gain is 0.0 until you paste yours over it.
    final TalonFXConfiguration talonFXCfg =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Coast) // easy to spin by hand
                    // positive shoots: clockwise from the motor side
                    .withInverted(InvertedValue.Clockwise_Positive))
            .withSlot0(new Slot0Configs().withKS(0.0).withKV(0.125).withKP(0.0))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(RotationsPerSecond.of(100.0))
                    .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(1000.0))
                    .withMotionMagicExpo_kV(
                        Volts.per(RotationsPerSecond).ofNative(0.119999997317791))
                    .withMotionMagicExpo_kA(
                        Volts.per(RotationsPerSecondPerSecond).ofNative(0.10000000149011612)));

    motor.getConfigurator().apply(talonFXCfg);
  }

  // Same setup as Arm. runRepeatedly runs the action every loop while the command
  // is scheduled. Every one of these is a hold: it never finishes on its own.

  /** Spin the flywheel at 25 rotations per second and hold it. Never finishes. */
  public Command runSlow() {
    return runRepeatedly(() -> setVelocity(25.0)).named("runSlow (hold)");
  }

  /** Spin the flywheel at 75 rotations per second and hold it. Never finishes. */
  public Command runFast() {
    return runRepeatedly(() -> setVelocity(75.0)).named("runFast (hold)");
  }

  /** Stop the flywheel and keep it stopped. Never finishes. */
  public Command stop() {
    return runRepeatedly(this::stopMotor).named("stop (hold)");
  }

  /** True when the flywheel is within tolerance of its target speed. */
  public boolean isAtTarget() {
    return getVelocity().isNear(getTargetVelocity(), tolerance);
  }

  /** Current measured flywheel speed. */
  public AngularVelocity getVelocity() {
    return motor.getVelocity().getValue();
  }

  /** Speed the flywheel is currently driving toward. */
  public AngularVelocity getTargetVelocity() {
    return velocityOut.getVelocityMeasure();
  }

  private void setVelocity(double rps) {
    motor.setControl(velocityOut.withVelocity(RotationsPerSecond.of(rps)));
  }

  /** Stop the motor. */
  private void stopMotor() {
    motor.stopMotor();
  }
}
