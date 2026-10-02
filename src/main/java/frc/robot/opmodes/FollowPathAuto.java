// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.opmodes;

import com.pathplanner.lib.path.PathPlannerPath;
import frc.robot.Robot;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.PeriodicOpMode;

/**
 * Drives whichever PathPlanner path is picked in the {@code Auto Path} drop-down.
 *
 * <p>The choice is read in {@link #start()}, not in the constructor. The constructor runs when the
 * mode is picked on the driver station, and the drop-down can still change after that.
 */
@Autonomous(name = "Follow Path")
public class FollowPathAuto extends PeriodicOpMode {
  private final Robot robot;
  private Command routine;

  public FollowPathAuto(Robot robot) {
    this.robot = robot;
  }

  @Override
  public void start() {
    PathPlannerPath path = robot.autoPath.getSelected();

    routine =
        Command.noRequirements(
                coroutine -> {
                  // Odometry starts wherever it was left. Put the robot at the path's first point,
                  // facing the path's starting rotation, so the plan and the robot agree.
                  path.getStartingHolonomicPose()
                      .ifPresent(pose -> robot.drivetrain.resetPose(pose));
                  coroutine.yield(); // give odometry one loop to report the new pose

                  coroutine.await(robot.drivetrain.followPath(path));
                })
            .named("Follow Path");

    Scheduler.getDefault().schedule(routine);
  }

  @Override
  public void end() {
    Scheduler.getDefault().cancel(routine);
  }
}
