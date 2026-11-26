// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.mechanisms;

import static org.wpilib.units.Units.Meters;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import first.robot.utils.SimUtil;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.util.Units;
import org.wpilib.simulation.SingleJointedArmSim;
import org.wpilib.system.RobotController;

/**
 * The arm's physics, for Simulate Robot Code only. {@link Arm} builds one when there is no real
 * arm, and from then on it plays the part of the arm: every loop it reads the voltage your code
 * asked the motor for, works out how a real arm would move, and tells the simulated motor and
 * CANcoder where the arm went.
 *
 * <p>Your code can't tell the difference. The closed loop you tuned in Tuner X still runs inside
 * the simulated Talon, against this arm.
 */
public class ArmSim {
  // The real arm, from its bill of materials. Change these and the simulated arm changes with them.
  static final double GEARING = 28.125; // 8:60 then 16:60: motor turns per arm turn
  static final double LENGTH_METERS = Arm.LENGTH.in(Meters); // pivot to tip
  static final double MASS_KG = 0.3;

  // WPILib's model of an arm swinging on one joint, under gravity.
  private final SingleJointedArmSim physics =
      new SingleJointedArmSim(
          DCMotor.getKrakenX44(1), // the motor driving it
          GEARING,
          SingleJointedArmSim.estimateMOI(LENGTH_METERS, MASS_KG), // how hard it is to swing
          LENGTH_METERS,
          Double.NEGATIVE_INFINITY, // no hard stops: it can swing all the way round
          Double.POSITIVE_INFINITY,
          true, // gravity
          Units.rotationsToRadians(0.75)); // starts hanging straight down

  private final TalonFX motor;
  private final TalonFXSimState motorSim;
  private final CANcoder encoder;
  private final CANcoderSimState encoderSim;
  private boolean encoderMatched;

  public ArmSim(TalonFX motor, CANcoder encoder) {
    this.motor = motor;
    this.motorSim = motor.getSimState();
    // The Talon models its own motor for current limiting. It assumes a Kraken X60 unless told,
    // and our bench uses an X44: leave this out and a fast-moving arm can run away.
    motorSim.setMotorType(TalonFXSimState.MotorType.KrakenX44);
    this.encoder = encoder;
    this.encoderSim = encoder.getSimState();
    SimUtil.everyLoop(motor, () -> update());
  }

  /** One loop of the simulation. The same steps CTRE's own simulations take. */
  private void update() {
    // Point the simulated CANcoder the way its config does. Until that works, the arm waits.
    if (!encoderMatched) {
      encoderMatched = SimUtil.matchCANcoderConfig(encoder);
      return;
    }

    // 1. Voltage in: what the motor is applying, from the battery we give it.
    motorSim.Orientation = SimUtil.orientationOf(motor);
    motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    encoderSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    physics.setInputVoltage(motorSim.getMotorVoltage());

    // 2. Physics: how the arm moves in one 20 ms loop.
    physics.update(0.020);

    // 3. Motion out: the CANcoder sits on the arm shaft, so it reads the arm itself.
    //    The motor's rotor spins GEARING times faster than the arm.
    double armRotations = Units.radiansToRotations(physics.getAngle());
    double armRotationsPerSecond = Units.radiansToRotations(physics.getVelocity());
    encoderSim.setRawPosition(armRotations);
    encoderSim.setVelocity(armRotationsPerSecond);
    motorSim.setRawRotorPosition(armRotations * GEARING);
    motorSim.setRotorVelocity(armRotationsPerSecond * GEARING);
  }

  /** The simulated arm's true angle, in rotations. For tests. */
  double getAngleRotations() {
    return Units.radiansToRotations(physics.getAngle());
  }
}
