// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.utils.TalonFXUtil;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;

/**
 * The arm. One TalonFX motor plus a CANcoder that measures the arm's angle.
 *
 * <p>New in this lesson: <b>Motion Magic</b> position control ({@link MotionMagicVoltage}). Same
 * gains as 3-PID, but instead of steering straight at the target, the motor follows a smooth speed
 * ramp: speed up, cruise, slow down. The ramp is called a motion profile, and it keeps the arm from
 * jerking.
 */
public class Arm extends Mechanism {
  // Target positions (rotations, 1.0 = one full turn).
  private static final double VERTICAL_POSITION = 0.25; // 90°  - stowed / safe transport
  private static final double HORIZONTAL_POSITION = 0.5; // 180° - ground intake

  // PID + feedforward gains.
  // TODO: CRITICAL - tune on the real robot before driving the arm under power.
  // Safe starting values: kG=0.2 (fights gravity), kS=0.2 (overcomes friction),
  //                       kP=160 (correction strength), kD=30 (smoothness).
  // If the arm jerks or moves too fast, make these smaller.
  private static final double kG = 0.0; // NEEDS TUNING - gravity feedforward
  private static final double kS = 0.0; // NEEDS TUNING - static friction feedforward
  private static final double kP = 0.0; // NEEDS TUNING - proportional gain
  private static final double kD = 0.0; // NEEDS TUNING - derivative gain

  // Motion Magic speed limits: how fast the arm may move and how quickly it may speed up.
  // TODO: CRITICAL - set these before running the arm. Good starting values:
  // cruise=2 rot/s, accel=4 rot/s².
  private static final double MOTION_MAGIC_CRUISE_VELOCITY = 0.0; // NEEDS SETTING - max rot/s
  private static final double MOTION_MAGIC_ACCELERATION = 0.0; // NEEDS SETTING - max rot/s²

  private final CANBus canivore = new CANBus("canivore");
  private final TalonFX motor = new TalonFX(31, canivore);
  private final CANcoder encoder = new CANcoder(32, canivore);

  // Moves the arm to a target angle along a smooth Motion Magic ramp.
  private final MotionMagicVoltage positionOut = new MotionMagicVoltage(0);

  public Arm() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast; // easy to move by hand
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine; // fights gravity automatically

    config.Slot0.kG = kG;
    config.Slot0.kS = kS;
    config.Slot0.kP = kP;
    config.Slot0.kD = kD;

    config.MotionMagic.MotionMagicCruiseVelocity = MOTION_MAGIC_CRUISE_VELOCITY;
    config.MotionMagic.MotionMagicAcceleration = MOTION_MAGIC_ACCELERATION;

    // Use the CANcoder for position, so PID works on the arm's real angle.
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
  //   arm.vertical().until(someCondition)   // ends when the condition turns true
  //
  // Every hold has "(hold)" in its name. Names show up on the dashboard and in logs, so if a
  // stuck sequence is sitting on a "(hold)", you found the bug.

  /** Move to the vertical (stowed) position and hold it. Never finishes. See the rule above. */
  public Command vertical() {
    return runRepeatedly(() -> setPosition(VERTICAL_POSITION)).named("vertical (hold)");
  }

  /** Move to the horizontal (ground intake) position and hold it. Never finishes. */
  public Command horizontal() {
    return runRepeatedly(() -> setPosition(HORIZONTAL_POSITION)).named("horizontal (hold)");
  }

  private void setPosition(double rotations) {
    motor.setControl(positionOut.withPosition(rotations));
  }
}
