// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.mechanisms;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Scheduler;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.system.Timer;

/**
 * Tests for the flywheel. The second test fakes a wheel speed through Phoenix 6 simulation, so
 * {@link Flywheel#isAtTarget} reads a number the test chose.
 */
class FlywheelTest {
  private final Scheduler scheduler = Scheduler.getDefault();
  private Flywheel flywheel;
  private TalonFXSimState motorSim;

  @BeforeEach
  void setUp() {
    // Start the simulated hardware layer, the same one Simulate Robot Code uses.
    assertTrue(HAL.initialize());
    flywheel = new Flywheel();

    // A second handle on CAN ID 21 reaches the same simulated motor as the one inside Flywheel,
    // so the mechanism's hardware can stay private.
    motorSim = new TalonFX(21, new CANBus("canivore")).getSimState();
    // Flywheel sets Clockwise_Positive. Tell the sim, or every speed it reports comes back
    // negative.
    motorSim.Orientation = ChassisReference.Clockwise_Positive;
  }

  @AfterEach
  void tearDown() {
    // The scheduler outlives each test. Clear it so one test's commands never leak into the next.
    scheduler.cancelAll();
  }

  @Test
  void runFastAsksFor75() {
    scheduler.schedule(flywheel.runFast());
    scheduler.run();

    assertEquals(75.0, flywheel.getTargetVelocity().in(RotationsPerSecond), 1e-9);
  }

  @Test
  void atTargetOnlyWithinHalfARotationPerSecond() {
    scheduler.schedule(flywheel.runFast());
    scheduler.run();

    motorSim.setRotorVelocity(74.8);
    // Simulated sensors update on their own schedule. Give the new speed time to arrive.
    Timer.delay(0.1);
    assertTrue(flywheel.isAtTarget());

    motorSim.setRotorVelocity(70.0);
    Timer.delay(0.1);
    assertFalse(flywheel.isAtTarget());
  }
}
