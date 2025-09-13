// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Flywheel;

public class RobotContainer {
  // Create an instance of the Arm subsystem
  private final Arm arm = new Arm();

  // Create an instance of the Flywheel subsystem
  private final Flywheel flywheel = new Flywheel();

  // Create a joystick to run subsystems off of
  private final CommandXboxController joystick = new CommandXboxController(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // When left trigger is pressed the arm will go to vertical position, and when not pressed it
    // will
    // stop
    joystick.leftTrigger().whileTrue(arm.vertical());

    // When right trigger is pressed the flywheel will run at a fast speed, and when not pressed it
    // will spin slow
    joystick.rightTrigger().onTrue(flywheel.runFast()).onFalse(flywheel.runSlow());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
