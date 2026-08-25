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
import com.ctre.phoenix6.controls.VoltageOut;
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
 * <p>New in this lesson: the arm offers <b>commands</b> (each method returns a
 * {@link Command}). Anything that wants to move the arm goes through
 * a command. That is how the scheduler keeps two things from fighting over the motor.
 *
 * <p>The commands still just push a voltage, so where the arm ends up depends on gravity and
 * friction. The next lesson makes the motor aim for a real target instead.
 */
public class Arm extends Mechanism {
  private final CANBus canivore = new CANBus("canivore");
  private final TalonFX motor = new TalonFX(31, canivore);
  private final CANcoder encoder = new CANcoder(32, canivore);

  // Pushes a set voltage at the motor. No sensors involved.
  private final VoltageOut voltageOut = new VoltageOut(0);

  public Arm() {
    final TalonFXConfiguration talonFXCfg =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Coast) // easy to move by hand
                    .withInverted(InvertedValue.CounterClockwise_Positive))
            .withMotionMagic(
                new MotionMagicConfigs()
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

  /** Push the arm at 3 volts and keep pushing. Never finishes. */
  public Command runSlow() {
    return runRepeatedly(() -> setVoltage(3.0)).named("runSlow (hold)");
  }

  /** Push the arm at 6 volts and keep pushing. Never finishes. */
  public Command runFast() {
    return runRepeatedly(() -> setVoltage(6.0)).named("runFast (hold)");
  }

  /** Stop the arm motor and keep it stopped. Never finishes. */
  public Command stop() {
    return runRepeatedly(this::stopMotor).named("stop (hold)");
  }

  private void setVoltage(double voltage) {
    motor.setControl(voltageOut.withOutput(voltage));
  }

  /** Stop the motor. */
  private void stopMotor() {
    motor.stopMotor();
  }
}
