// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.mechanisms;

import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.RotationsPerSecondPerSecond;
import static org.wpilib.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;

/**
 * The arm. One TalonFX motor plus a CANcoder that measures the arm's angle.
 *
 * <p>New in this lesson: the arm goes closed loop. Instead of pushing a voltage and hoping, each
 * command names a target angle, and the motor drives to it along a <b>Motion Magic</b> profile
 * ({@link MotionMagicVoltage}) that speeds up, cruises and slows down so the arm does not jerk.
 *
 * <p>The gains are not typed here. They are pasted out of Phoenix Tuner X, where you measured them
 * in Workshop 1.
 *
 * <p>There is also no stop command anymore. A {@code Mechanism} with nothing commanding it runs an
 * idle default command on its own.
 */
public class Arm extends Mechanism {
  private final CANBus canivore = new CANBus("canivore");
  private final TalonFX motor = new TalonFX(31, canivore);
  private final CANcoder encoder = new CANcoder(32, canivore);

  // Moves the arm to a target angle along a smooth Motion Magic ramp.
  private final MotionMagicVoltage positionOut = new MotionMagicVoltage(0);

  public Arm() {
    // Pasted from Tuner X. Every gain is 0.0 until you paste yours over it.
    final TalonFXConfiguration talonFXCfg =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Coast) // easy to move by hand
                    .withInverted(InvertedValue.CounterClockwise_Positive))
            .withSlot0(
                new Slot0Configs()
                    .withKG(0.0)
                    .withKS(0.0)
                    .withKP(0.0)
                    .withKD(0.0)
                    .withGravityType(GravityTypeValue.Arm_Cosine))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(RotationsPerSecond.of(0.0))
                    .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(0.0))
                    .withMotionMagicExpo_kV(
                        Volts.per(RotationsPerSecond).ofNative(0.119999997317791))
                    .withMotionMagicExpo_kA(
                        Volts.per(RotationsPerSecondPerSecond).ofNative(0.10000000149011612)))
            .withFeedback(
                new FeedbackConfigs()
                    .withFeedbackRemoteSensorID(32)
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder));

    motor.getConfigurator().apply(talonFXCfg);
  }

  // Each command uses runRepeatedly, which runs its action every loop while the
  // command is scheduled. Every one of them is a hold: it never finishes on its own.

  /**
   * Move to vertical, 0.25 rotations or 90 degrees, and hold it there. This is the stowed
   * position for transport. Never finishes.
   */
  public Command vertical() {
    return runRepeatedly(() -> setPosition(0.25)).named("vertical (hold)");
  }

  /**
   * Move to horizontal, 0.5 rotations or 180 degrees, and hold it there. This is the ground
   * intake position. Never finishes.
   */
  public Command horizontal() {
    return runRepeatedly(() -> setPosition(0.5)).named("horizontal (hold)");
  }

  private void setPosition(double rotations) {
    motor.setControl(positionOut.withPosition(rotations));
  }
}
