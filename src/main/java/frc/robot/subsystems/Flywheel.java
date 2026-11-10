// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.RotationsPerSecondPerSecond;
import static org.wpilib.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.interpolation.InterpolatingDoubleTreeMap;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.telemetry.TelemetryTable;

/**
 * A flywheel that picks its speed from how far the robot is from the goal.
 *
 * <p>The speeds live in a lookup table. You measure a few distances, enter the speed that scores at
 * each one, and {@link InterpolatingDoubleTreeMap} fills in everything between them. Every loop,
 * {@link #distanceShoot()} checks where the robot is, looks up the speed for that distance, and
 * asks the motor for it.
 */
public class Flywheel implements Mechanism {
  // The point we are shooting at, from the blue alliance corner, in meters. Set the real goal.
  private static final Translation2d TARGET = new Translation2d(3, 5);

  private final CANBus canivore = new CANBus("canivore");
  private final TalonFX leader = new TalonFX(21, canivore);
  private final TalonFX follower = new TalonFX(22, canivore);

  // Asks the motor to ramp to a target speed instead of jumping to it.
  private final MotionMagicVelocityVoltage velocityOut = new MotionMagicVelocityVoltage(0);

  private final DriveMechanism drivetrain;

  // Distance in meters -> flywheel speed in rotations per second. Gaps are filled in for you.
  private final InterpolatingDoubleTreeMap table = new InterpolatingDoubleTreeMap();

  // Everything logged here lands under Flywheel/, live on the dashboard and in the log file.
  private final TelemetryTable telemetry = Telemetry.getTable(getName());

  /**
   * @param drivetrain the drivetrain whose pose gives the distance to the goal
   */
  public Flywheel(DriveMechanism drivetrain) {
    this.drivetrain = drivetrain;

    // Placeholders. Replace every row with speeds you measured on your own robot.
    table.put(0.0, 0.0);
    table.put(1.0, 10.0);
    table.put(2.0, 30.0);
    table.put(3.0, 60.0);

    // Pasted from Tuner X. Every gain is 0.0 until you paste yours over it.
    final TalonFXConfiguration talonFXCfg =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Coast) // easy to spin by hand
                    .withInverted(InvertedValue.CounterClockwise_Positive))
            .withSlot0(new Slot0Configs().withKS(0.0).withKV(0.0).withKP(0.0))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(RotationsPerSecond.of(0.0))
                    .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(0.0))
                    .withMotionMagicExpo_kV(
                        Volts.per(RotationsPerSecond).ofNative(0.119999997317791))
                    .withMotionMagicExpo_kA(
                        Volts.per(RotationsPerSecondPerSecond).ofNative(0.10000000149011612)));

    leader.getConfigurator().apply(talonFXCfg);

    // The follower copies the leader, spinning the opposite direction.
    follower.setControl(new Follower(leader.getDeviceID(), MotorAlignmentValue.Opposed));
  }

  /** Keep setting the speed from the live distance to the goal. A hold: it never finishes. */
  public Command distanceShoot() {
    return runRepeatedly(() -> setVelocity(table.get(distanceToTarget()))).named("distanceShoot");
  }

  /** Stop the flywheel and keep it stopped. Never finishes. */
  public Command stop() {
    return runRepeatedly(() -> stopMotor()).named("stop");
  }

  /** Distance in meters from where the robot thinks it is to the goal. */
  private double distanceToTarget() {
    double distance = drivetrain.getPose().getTranslation().getDistance(TARGET);
    telemetry.log("DistanceToTargetMeters", distance);
    return distance;
  }

  private void setVelocity(double rps) {
    telemetry.log("TargetVelocityRps", rps);
    leader.setControl(velocityOut.withVelocity(RotationsPerSecond.of(rps)));
  }

  private void stopMotor() {
    leader.stopMotor();
  }
}
