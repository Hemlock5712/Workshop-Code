// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.opmodes;

import static org.wpilib.units.Units.Seconds;

import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.Robot;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.PeriodicOpMode;

/**
 * Drives forward off the starting line for a second and a half, then stops.
 *
 * <p>A timed drive is crude on purpose. It proves the mode list, the scheduler and the drivetrain
 * agree with each other. It does not know where the robot ended up. The PathPlanner lesson replaces
 * the timer with a drawn path. See frc5712.com.
 */
@Autonomous(name = "Leave Start")
public class LeaveStartAuto extends PeriodicOpMode {
  private final Command routine;

  public LeaveStartAuto(Robot robot) {
    // Robot-centric: X is the robot's own forward, so the starting heading sets the direction.
    final var forward = new SwerveRequest.RobotCentric().withVelocityX(1.0); // meters per second
    final var stopped = new SwerveRequest.RobotCentric(); // every speed is zero

    // setControl latches a request. The drivetrain keeps applying the last one until something
    // sends another, and this OpMode sets no default command. So the routine sends the zero itself,
    // on the way out and when it is canceled partway.
    routine =
        robot
            .drivetrain
            .run(
                coroutine -> {
                  robot.drivetrain.setControl(forward);
                  coroutine.wait(Seconds.of(1.5));
                  robot.drivetrain.setControl(stopped);
                })
            .whenCanceled(() -> robot.drivetrain.setControl(stopped))
            .named("Leave Start");
  }

  /** No trigger owns this routine, so the OpMode starts and stops it. */
  @Override
  public void start() {
    Scheduler.getDefault().schedule(routine);
  }

  @Override
  public void end() {
    Scheduler.getDefault().cancel(routine);
  }
}
