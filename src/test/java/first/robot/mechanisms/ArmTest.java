// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.mechanisms;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Rotations;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.hardware.hal.HAL;

/**
 * Tests for the arm's commands. No motor moves: each test schedules a command, runs the scheduler
 * once by hand, and checks what the arm asked the motor for.
 */
class ArmTest {
  private final Scheduler scheduler = Scheduler.getDefault();
  private Arm arm;

  @BeforeEach
  void setUp() {
    // Start the simulated hardware layer, the same one Simulate Robot Code uses.
    assertTrue(HAL.initialize());
    arm = new Arm();
  }

  @AfterEach
  void tearDown() {
    // The scheduler outlives each test. Clear it so one test's commands never leak into the next.
    scheduler.cancelAll();
  }

  @Test
  void verticalAsksForAQuarterTurn() {
    scheduler.schedule(arm.vertical());
    scheduler.run();

    assertEquals(0.25, arm.getTargetPosition().in(Rotations), 1e-9);
  }

  @Test
  void horizontalTakesTheArmFromVertical() {
    Command vertical = arm.vertical();
    Command horizontal = arm.horizontal();

    scheduler.schedule(vertical);
    scheduler.run();
    scheduler.schedule(horizontal);
    scheduler.run();

    // One command per mechanism. The newer one wins and the older one is canceled.
    assertFalse(scheduler.isRunning(vertical));
    assertTrue(scheduler.isRunning(horizontal));
    assertEquals(0.5, arm.getTargetPosition().in(Rotations), 1e-9);
  }
}
