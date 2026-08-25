// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.mechanisms;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;

/**
 * The arm. One TalonFX motor plus a CANcoder that measures the arm's angle.
 *
 * <p>New in this lesson: closed-loop <b>position</b> control ({@link PositionVoltage}). Instead of
 * pushing a voltage and hoping, each command picks a target angle. The motor's PID controller
 * steers to that angle and holds it.
 *
 * <p>Also new: there is no stop command anymore. A {@code Mechanism} with nothing commanding it
 * runs an idle default command on its own, so we don't have to write one.
 *
 * <p>The next lesson (mech-4-MotionMagic) adds a motion profile on top of the same gains.
 */
public class Arm extends Mechanism {
  // Target positions (rotations, 1.0 = one full turn).
  private static final double VERTICAL_POSITION = 0.25; // 90 degrees, stowed for transport
  private static final double HORIZONTAL_POSITION = 0.5; // 180 degrees, ground intake

  // PID + feedforward gains.
  // TODO: tune these on the real robot before the arm moves under power.
  // Safe starting values: kG=0.2 (fights gravity), kS=0.2 (overcomes friction),
  //                       kP=160 (correction strength), kD=30 (smoothness).
  // If the arm jerks or moves too fast, make these smaller.
  private static final double kG = 0.0; // NEEDS TUNING, gravity feedforward
  private static final double kS = 0.0; // NEEDS TUNING, static friction feedforward
  private static final double kP = 0.0; // NEEDS TUNING, proportional gain
  private static final double kD = 0.0; // NEEDS TUNING, derivative gain

  private final CANBus canivore = new CANBus("canivore");
  private final TalonFX motor = new TalonFX(31, canivore);
  private final CANcoder encoder = new CANcoder(32, canivore);

  // Asks the motor's PID to move the arm to a target angle and hold it.
  private final PositionVoltage positionOut = new PositionVoltage(0);

  public Arm() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast; // easy to move by hand
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine; // fights gravity automatically
    config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

    config.Slot0.kG = kG;
    config.Slot0.kS = kS;
    config.Slot0.kP = kP;
    config.Slot0.kD = kD;

    // Use the CANcoder for position, so PID works on the arm's real angle.
    config.Feedback.withRemoteCANcoder(encoder);

    motor.getConfigurator().apply(config);
  }

  // Each command below uses runRepeatedly, which runs its action every loop while the command is
  // scheduled. Re-sending the request every loop also restores it if a motor controller reboots.
  //
  // One rule: these commands are all holds. A hold never finishes, so never make anything
  // wait for a hold. A hold inside Command.sequence sticks there forever. If a step needs an
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
