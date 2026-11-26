// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.mechanisms;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import first.robot.utils.SimUtil;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.math.util.Units;
import org.wpilib.system.RobotController;

/**
 * The flywheel's physics, for Simulate Robot Code only. {@link Flywheel} builds one when there is
 * no real flywheel, and from then on it plays the part of the wheel: every loop it reads the
 * voltage your code asked the motor for, works out how fast a real wheel would spin, and tells the
 * simulated motor.
 *
 * <p>Your code can't tell the difference. The closed loop you tuned in Tuner X still runs inside
 * the simulated Talon, against this wheel.
 */
public class FlywheelSim {
  // The real flywheel. The motor's speed is the wheel speed your code commands, so the gearing here
  // is 1, matching the config.
  private static final double GEARING = 1.0;
  private static final double MOMENT_OF_INERTIA = 0.002; // kg·m², how hard it is to spin up

  private static final DCMotor MOTOR = DCMotor.getKrakenX44(1);

  // WPILib's model of a spinning wheel. (Written out in full because it shares this class's name.)
  private final org.wpilib.simulation.FlywheelSim physics =
      new org.wpilib.simulation.FlywheelSim(
          Models.flywheelFromPhysicalConstants(MOTOR, MOMENT_OF_INERTIA, GEARING), MOTOR);

  private final TalonFX motor;
  private final TalonFXSimState motorSim;
  private double wheelRotations;

  public FlywheelSim(TalonFX motor) {
    this.motor = motor;
    this.motorSim = motor.getSimState();
    // The Talon models its own motor for current limiting. It assumes a Kraken X60 unless told,
    // and our bench uses an X44: leave this out and a fast-moving flywheel can run away.
    motorSim.setMotorType(TalonFXSimState.MotorType.KrakenX44);
    SimUtil.everyLoop(motor, () -> update());
  }

  /** One loop of the simulation. The same steps CTRE's own simulations take. */
  private void update() {
    // 1. Voltage in: what the motor is applying, from the battery we give it.
    motorSim.Orientation = SimUtil.orientationOf(motor);
    motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    physics.setInputVoltage(motorSim.getMotorVoltage());

    // 2. Physics: how the wheel speeds up or slows down in one 20 ms loop.
    physics.update(0.020);

    // 3. Motion out: the rotor spins GEARING times faster than the wheel.
    double wheelRotationsPerSecond = Units.radiansToRotations(physics.getAngularVelocity());
    wheelRotations += wheelRotationsPerSecond * 0.020;
    motorSim.setRawRotorPosition(wheelRotations * GEARING);
    motorSim.setRotorVelocity(wheelRotationsPerSecond * GEARING);
  }

  /** The simulated wheel's true speed, in rotations per second. For tests. */
  double getVelocityRotationsPerSecond() {
    return Units.radiansToRotations(physics.getAngularVelocity());
  }
}
