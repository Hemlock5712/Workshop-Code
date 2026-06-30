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
 * Publishes the swerve drivetrain state to NetworkTables. Register it with {@code
 * drivetrain.registerTelemetry(telemetry::telemeterize)} and CTRE invokes {@link
 * #telemeterize(SwerveDriveState)} from the odometry thread every time a new state is produced (250
 * Hz on CAN FD).
 *
 * <p>This is the project's <b>logging surface</b>. There is no AdvantageKit in this template; the
 * "logging-only" story is: publish the things you care about to NetworkTables here, and {@link
 * org.wpilib.system.DataLogManager} (started in {@link frc.robot.Robot}) captures every NT value
 * change into a {@code .wpilog} on disk. So everything published below is visible <i>live</i> in
 * Glass / AdvantageScope and also recorded for after-the-match analysis. The keys land under {@code
 * Drivetrain/*} on NT and {@code NT:/Drivetrain/*} in the log.
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

  // Per-module measured states, commanded targets, and odometry positions. AdvantageScope renders
  // SwerveModuleVelocity[]/SwerveModulePosition[] natively on the swerve widget.
  private final StructArrayPublisher<SwerveModuleVelocity> moduleStates =
      table.getStructArrayTopic("ModuleStates", SwerveModuleVelocity.struct).publish();
  private final StructArrayPublisher<SwerveModuleVelocity> moduleTargets =
      table.getStructArrayTopic("ModuleTargets", SwerveModuleVelocity.struct).publish();
  private final StructArrayPublisher<SwerveModulePosition> modulePositions =
      table.getStructArrayTopic("ModulePositions", SwerveModulePosition.struct).publish();

  // Scalars that are handy as plot traces / health checks.
  private final DoublePublisher translationSpeed =
      table.getDoubleTopic("TranslationSpeedMps").publish();
  private final DoublePublisher rotationSpeed =
      table.getDoubleTopic("RotationSpeedRadPerSec").publish();
  private final DoublePublisher odometryPeriod =
      table.getDoubleTopic("OdometryPeriodSeconds").publish();
  private final DoublePublisher odometryFrequency =
      table.getDoubleTopic("OdometryFrequencyHz").publish();

  /**
   * Publishes one drivetrain state. Called by CTRE on the odometry thread; the NetworkTables
   * publishers are thread-safe, so no locking is needed.
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
