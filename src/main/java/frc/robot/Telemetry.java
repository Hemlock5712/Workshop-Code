// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.networktables.DoublePublisher;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.StructArrayPublisher;
import org.wpilib.networktables.StructPublisher;

/**
 * Sends the drivetrain's state to NetworkTables, so you can watch it live in Glass or
 * AdvantageScope. CTRE calls {@link #telemeterize(SwerveDriveState)} with fresh data up to 250
 * times per second.
 *
 * <p>Everything lands under {@code Drivetrain/*}. Want to watch something else? Add a publisher
 * field and set it in telemeterize.
 */
public class Telemetry {
  private final NetworkTable table = NetworkTableInstance.getDefault().getTable("Drivetrain");

  // Pose + velocity (the things you almost always want).
  private final StructPublisher<Pose2d> pose =
      table.getStructTopic("Pose", Pose2d.struct).publish();
  private final StructPublisher<ChassisVelocities> velocity =
      table.getStructTopic("Velocity", ChassisVelocities.struct).publish();
  private final StructPublisher<Rotation2d> rawHeading =
      table.getStructTopic("RawHeading", Rotation2d.struct).publish();

  // Each wheel module: measured speeds, target speeds, and positions. AdvantageScope's swerve
  // widget can draw these directly.
  private final StructArrayPublisher<SwerveModuleVelocity> moduleStates =
      table.getStructArrayTopic("ModuleStates", SwerveModuleVelocity.struct).publish();
  private final StructArrayPublisher<SwerveModuleVelocity> moduleTargets =
      table.getStructArrayTopic("ModuleTargets", SwerveModuleVelocity.struct).publish();
  private final StructArrayPublisher<SwerveModulePosition> modulePositions =
      table.getStructArrayTopic("ModulePositions", SwerveModulePosition.struct).publish();

  // Single numbers that are handy to plot.
  private final DoublePublisher translationSpeed =
      table.getDoubleTopic("TranslationSpeedMps").publish();
  private final DoublePublisher rotationSpeed =
      table.getDoubleTopic("RotationSpeedRadPerSec").publish();
  private final DoublePublisher odometryPeriod =
      table.getDoubleTopic("OdometryPeriodSeconds").publish();
  private final DoublePublisher odometryFrequency =
      table.getDoubleTopic("OdometryFrequencyHz").publish();

  /**
   * Publishes one drivetrain state. CTRE calls this from a background thread, which is fine because
   * NetworkTables publishers are safe to use from any thread.
   *
   * @param state the latest swerve drive state
   */
  public void telemeterize(SwerveDriveState state) {
    pose.set(state.Pose);
    velocity.set(state.Velocity);
    rawHeading.set(state.RawHeading);

    moduleStates.set(state.ModuleVelocities);
    moduleTargets.set(state.ModuleTargets);
    modulePositions.set(state.ModulePositions);

    translationSpeed.set(Math.hypot(state.Velocity.vx, state.Velocity.vy));
    rotationSpeed.set(state.Velocity.omega);
    odometryPeriod.set(state.OdometryPeriod);
    odometryFrequency.set(state.OdometryPeriod > 0 ? 1.0 / state.OdometryPeriod : 0.0);
  }
}
