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
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.interpolation.InterpolatingDoubleTreeMap;
import org.wpilib.networktables.DoublePublisher;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;

/**
 * A "dynamic" flywheel: it picks its shooting speed from how far the robot is from the goal. (Same
 * flywheel as the mechanism track, plus the distance lookup.)
 *
 * <p>The speeds live in a lookup table. You measure a few distances, enter the speed that works at
 * each one, and {@link InterpolatingDoubleTreeMap} fills in everything between them. Every loop,
 * the {@link #distanceShoot()} command checks where the robot is, looks up the speed for that
 * distance, and asks the motor for it.
 *
 * <p>A {@code Mechanism} has no periodic() method, so that measure-then-set work happens inside the
 * command itself, which runs every loop.
 */
public class Flywheel extends Mechanism {
  // Field point we are shooting at, blue-alliance origin (meters). TODO: set the real goal.
  private static final Translation2d TARGET = new Translation2d(3, 5);

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

  private final DriveMechanism drivetrain;

  // distance (meters) -> flywheel speed (rotations/second). Gaps are filled in automatically.
  private final InterpolatingDoubleTreeMap table = new InterpolatingDoubleTreeMap();

  // Publish live numbers to NetworkTables. DataLogManager also records them to the log file.
  private final NetworkTable telemetry = NetworkTableInstance.getDefault().getTable("Flywheel");
  private final DoublePublisher distancePublisher =
      telemetry.getDoubleTopic("DistanceToTargetMeters").publish();
  private final DoublePublisher targetVelocityPublisher =
      telemetry.getDoubleTopic("TargetVelocityRps").publish();

  public Flywheel(DriveMechanism drivetrain) {
    this.drivetrain = drivetrain;

    // Build the distance -> speed table. Tune these points with real test shots.
    table.put(0.0, 0.0);
    table.put(1.0, 10.0);
    table.put(2.0, 30.0);
    table.put(3.0, 60.0);

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

  // Both commands below are HOLDS: runRepeatedly runs the action every loop and never finishes.
  // Never make a sequence wait on a hold. Need an ending? Add it where you use the command:
  // flywheel.distanceShoot().until(someCondition). The "(hold)" in each name shows up on the
  // dashboard and in logs - if a stuck routine is sitting on a "(hold)", you found the bug.

  /**
   * Keep setting the flywheel speed from the live distance to the target. A hold - it never
   * finishes on its own. Bind it with {@code whileTrue} so it stops when the button is released.
   */
  public Command distanceShoot() {
    return runRepeatedly(() -> setVelocity(table.get(distanceToTarget())))
        .named("distanceShoot (hold)");
  }

  /** Stop the flywheel and keep it stopped. Never finishes. */
  public Command stop() {
    return runRepeatedly(leader::stopMotor).named("stop (hold)");
  }

  /** Distance (meters) from where the robot thinks it is to the target. */
  private double distanceToTarget() {
    double distance = drivetrain.getPose().getTranslation().getDistance(TARGET);
    distancePublisher.set(distance);
    return distance;
  }

  private void setVelocity(double rps) {
    targetVelocityPublisher.set(rps);
    leader.setControl(velocityOut.withVelocity(RotationsPerSecond.of(rps)));
  }
}
