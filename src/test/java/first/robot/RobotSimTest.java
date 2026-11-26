// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.wpilib.framework.RobotBase;
import org.wpilib.hardware.hal.RobotMode;

/** Starts the whole robot program in simulation and checks that it enables. */
class RobotSimTest {
  @Test
  void teleopEnables() throws InterruptedException {
    SimRobot.start();
    SimRobot.enable(RobotMode.TELEOPERATED, "Teleop");

    assertTrue(RobotBase.isTeleopEnabled());
    assertEquals("Teleop", RobotBase.getOpMode());

    SimRobot.disable();
  }
}
