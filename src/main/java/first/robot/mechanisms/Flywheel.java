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
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;

/**
 * The flywheel. One TalonFX motor (CAN 21). No CANcoder: the encoder inside the motor already
 * measures speed.
 *
 * <p>Same idea as {@link Arm}. The flywheel offers commands now. The commands still push plain
 * voltage. The next lesson switches to real velocity control.
 */
public class Flywheel extends Mechanism {
  private final CANBus canivore = new CANBus("canivore");
  private final TalonFX motor = new TalonFX(21, canivore);

  // Pushes a set voltage at the motor. No sensors involved.
  private final VoltageOut voltageOut = new VoltageOut(0);

  public Flywheel() {
    final TalonFXConfiguration talonFXCfg =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Coast) // easy to spin by hand
                    // positive shoots: clockwise from the motor side
                    .withInverted(InvertedValue.Clockwise_Positive))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicExpo_kV(
                        Volts.per(RotationsPerSecond).ofNative(0.119999997317791))
                    .withMotionMagicExpo_kA(
                        Volts.per(RotationsPerSecondPerSecond).ofNative(0.10000000149011612)));

    motor.getConfigurator().apply(talonFXCfg);
  }

  // Same setup as Arm. runRepeatedly runs the action every loop while the command
  // is scheduled. Every one of these is a hold: it never finishes on its own.

  /** Spin the flywheel at 3 volts and hold it there. Never finishes. */
  public Command runSlow() {
    return runRepeatedly(() -> setVoltage(3.0)).named("runSlow (hold)");
  }

  /** Spin the flywheel at 6 volts and hold it there. Never finishes. */
  public Command runFast() {
    return runRepeatedly(() -> setVoltage(6.0)).named("runFast (hold)");
  }

  /** Stop the flywheel and keep it stopped. Never finishes. */
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
