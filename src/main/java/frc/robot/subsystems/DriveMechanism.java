// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.Telemetry;
import frc.robot.generated.TunerConstants;
import java.util.function.Supplier;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;

/**
 * The drivetrain's {@code Mechanism}. The swerve drivetrain class already extends CTRE's generated
 * class, and a Java class can only extend one thing. So this class owns the drivetrain and offers
 * its commands to the rest of the robot.
 */
public class DriveMechanism extends Mechanism {
  // TunerConstants comes from the Tuner X swerve generator. The checked-in file is an EXAMPLE
  // with fake device IDs and gains - regenerate it from Tuner X for your own robot.
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  // Sends drivetrain data to NetworkTables so you can watch it live. See Telemetry.
  private final Telemetry telemetry = new Telemetry();

  public DriveMechanism() {
    super("Drivetrain");
    // Every loop, check which alliance we are on so "forward" faces the right way.
    Scheduler.getDefault().addPeriodic(drivetrain::applyOperatorPerspective);
    // CTRE feeds Telemetry fresh data up to 250 times per second.
    drivetrain.registerTelemetry(telemetry::telemeterize);
  }

  /** Returns a command that keeps sending the given control request to the drivetrain. */
  public Command applyRequest(Supplier<SwerveRequest> request) {
    return runRepeatedly(() -> drivetrain.setControl(request.get())).named("applyRequest");
  }

  /** Resets the field-centric heading so "forward" matches the driver's current facing. */
  public Command seedFieldCentric() {
    return run(coroutine -> {
          drivetrain.seedFieldCentric();
        })
        .named("seedFieldCentric");
  }

  /**
   * Sends one control request straight to the drivetrain. A command that already requires this
   * mechanism (like {@code DriveToPoint} in a later lesson) uses this to drive.
   */
  public void setControl(SwerveRequest request) {
    drivetrain.setControl(request);
  }

  /**
   * The robot's position on the field, from odometry. (0, 0) is always the blue alliance corner. It
   * does not flip when you are on red.
   */
  public Pose2d getPose() {
    return drivetrain.getState().Pose;
  }

  /**
   * How fast the robot is moving across the field. The drivetrain measures speed relative to the
   * robot, so this turns it into field directions.
   */
  public ChassisVelocities getFieldVelocity() {
    var state = drivetrain.getState();
    return state.Velocity.toFieldRelative(state.Pose.getRotation());
  }

  /**
   * Feeds a camera position estimate into the drivetrain so it can correct odometry. The Limelight
   * subsystem (added in a later lesson) calls this.
   *
   * @param visionRobotPose the robot position the camera measured (blue-alliance origin)
   * @param timestampSeconds when the picture was taken, on the {@code
   *     Utils.getCurrentTimeSeconds()} clock
   * @param stdDevs how much to trust the measurement [x, y, theta]ᵀ (meters, radians) - bigger
   *     numbers mean trust it less
   */
  public void addVisionMeasurement(
      Pose2d visionRobotPose, double timestampSeconds, Matrix<N3, N1> stdDevs) {
    drivetrain.addVisionMeasurement(visionRobotPose, timestampSeconds, stdDevs);
  }
}
