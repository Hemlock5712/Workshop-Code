// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.utils;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AppliedRotorPolarityValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.ChassisReference;
import java.util.HashMap;
import java.util.Map;
import org.wpilib.command3.Scheduler;

/**
 * The CTRE plumbing a mechanism simulation needs, so the {@code ...Sim} classes can stay about the
 * physics.
 */
public final class SimUtil {
  private SimUtil() {}

  /**
   * Which way the motor is mounted, for its {@code SimState.Orientation}. CTRE's own swerve
   * simulation uses Clockwise_Positive when the motor is inverted. This reads the direction the
   * Talon reports it is applying, which follows the Inverted config.
   */
  public static ChassisReference orientationOf(TalonFX motor) {
    return motor.getAppliedRotorPolarity().getValue()
            == AppliedRotorPolarityValue.PositiveIsClockwise
        ? ChassisReference.Clockwise_Positive
        : ChassisReference.CounterClockwise_Positive;
  }

  /**
   * Copies a CANcoder's sensor direction and magnet offset onto its {@code SimState}, the way
   * CTRE's own swerve simulation does, so the simulated CANcoder reports the true angle. Reading a
   * config can time out while the simulator is still starting, so call this every loop until it
   * returns true.
   */
  public static boolean matchCANcoderConfig(CANcoder encoder) {
    var magnet = new MagnetSensorConfigs();
    if (!encoder.getConfigurator().refresh(magnet, 0.05).isOK()) {
      return false;
    }
    var encoderSim = encoder.getSimState();
    encoderSim.Orientation =
        magnet.SensorDirection == SensorDirectionValue.Clockwise_Positive
            ? ChassisReference.Clockwise_Positive
            : ChassisReference.CounterClockwise_Positive;
    encoderSim.SensorOffset = magnet.MagnetOffset;
    return true;
  }

  // The newest simulation for each motor. Building a mechanism twice (every JUnit test does)
  // retires the old simulation instead of running two against the same device.
  private static final Map<String, Runnable> newest = new HashMap<>();

  /**
   * Runs {@code step} once every robot loop, for as long as it is the newest simulation of this
   * motor. Call it from a constructor that runs before any OpMode: started inside an OpMode or a
   * command, the scheduler stops it when that OpMode or command ends.
   */
  public static void everyLoop(TalonFX motor, Runnable step) {
    String device = motor.getNetwork().getName() + ":" + motor.getDeviceID();
    newest.put(device, step);
    Scheduler.getDefault()
        .sideload(
            coroutine -> {
              while (newest.get(device) == step) {
                step.run();
                coroutine.yield();
              }
            });
  }
}
