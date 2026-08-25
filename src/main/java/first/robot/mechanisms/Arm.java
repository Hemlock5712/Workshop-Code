// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.mechanisms;

import static org.wpilib.units.Units.Degrees;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.units.measure.Angle;

/**
 * The arm. One TalonFX motor plus a CANcoder that measures the arm's angle.
 *
 * <p>The motor uses Motion Magic position control ({@link MotionMagicVoltage}), added in
 * mech-4-MotionMagic: it follows a smooth speed ramp to the target angle instead of jerking toward it.
 *
 * <p>New in this lesson (mech-5-ReadingState): the <b>read side</b>. {@link #getPosition} tells you
 * where the arm is, {@link #getTargetPosition} tells you where it is headed, and {@link
 * #isAtTarget} tells you whether it has arrived. That last check is how a hold gets an ending:
 * {@code arm.vertical().until(arm::isAtTarget)} finishes when the arm is really there.
 */
public class Arm extends Mechanism {
  // Target positions (rotations, 1.0 = one full turn).
  private static final double VERTICAL_POSITION = 0.25; // 90 degrees, stowed for transport
  private static final double HORIZONTAL_POSITION = 0.5; // 180 degrees, ground intake

  // How close counts as "at target".
  private static final double POSITION_TOLERANCE_DEGREES = 1.0;

  // PID + feedforward gains.
  // TODO: tune these on the real robot before the arm moves under power.
  // Safe starting values: kG=0.2 (fights gravity), kS=0.2 (overcomes friction),
  //                       kP=160 (correction strength), kD=30 (smoothness).
  // If the arm jerks or moves too fast, make these smaller.
  private static final double kG = 0.0; // NEEDS TUNING, gravity feedforward
  private static final double kS = 0.0; // NEEDS TUNING, static friction feedforward
  private static final double kP = 0.0; // NEEDS TUNING, proportional gain
  private static final double kD = 0.0; // NEEDS TUNING, derivative gain

  // Motion Magic speed limits: how fast the arm may move and how quickly it may speed up.
  // TODO: set these before the arm moves under power. Good starting values:
  // cruise=2 rot/s, accel=4 rot/s².
  private static final double MOTION_MAGIC_CRUISE_VELOCITY = 0.0; // NEEDS SETTING, max rot/s
  private static final double MOTION_MAGIC_ACCELERATION = 0.0; // NEEDS SETTING, max rot/s²

  private final CANBus canivore = new CANBus("canivore");
  private final TalonFX motor = new TalonFX(31, canivore);
  private final CANcoder encoder = new CANcoder(32, canivore);

  // Moves the arm to a target angle along a smooth Motion Magic ramp.
  private final MotionMagicVoltage positionOut = new MotionMagicVoltage(0);

  private final Angle tolerance = Degrees.of(POSITION_TOLERANCE_DEGREES);

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

    motor.getConfigurator().apply(config);
  }

  // Each command below uses runRepeatedly, which runs its action every loop while the command is
  // scheduled. Re-sending the request every loop also restores it if a motor controller reboots.
  //
  // One rule: these commands are all holds. A hold never finishes, so never make anything
  // wait for a hold. A hold inside Command.sequence sticks there forever. If a step needs an
  // ending, add one where you use the command instead of writing a new method here:
  //
  //   arm.vertical().until(arm::isAtTarget)   // ends when the arm arrives
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

  /** True when the arm has reached its target angle. */
  public boolean isAtTarget() {
    return getPosition().isNear(getTargetPosition(), tolerance);
  }

  /** Current measured arm angle. */
  public Angle getPosition() {
    return encoder.getPosition().getValue();
  }

  /** Angle the arm is currently driving toward. */
  public Angle getTargetPosition() {
    return positionOut.getPositionMeasure();
  }

  private void setPosition(double rotations) {
    motor.setControl(positionOut.withPosition(rotations));
  }
}
