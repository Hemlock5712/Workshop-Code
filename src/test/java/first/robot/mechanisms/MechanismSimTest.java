// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.mechanisms;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;
import static org.wpilib.units.Units.Meters;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import first.robot.utils.MechanismViews;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.wpilib.backend.NetworkTablesTelemetryBackend;
import org.wpilib.command3.Scheduler;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.math.system.DCMotor;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.telemetry.TelemetryRegistry;

/**
 * Runs the real Phoenix closed loop against ArmSim and FlywheelSim, in real time, the way Simulate
 * Robot Code does, and reads back what the Mechanism2d pictures publish. Device IDs from 41 up,
 * with test gains, so nothing collides with the bench mechanisms and their 0.0 placeholder gains.
 */
class MechanismSimTest {
  private static final CANBus bus = new CANBus("canivore");

  @BeforeAll
  static void enable() throws InterruptedException {
    assertTrue(HAL.initialize());
    // RobotBase does this when a robot starts. Tests have no robot, so do it here: it sends
    // Telemetry.log to NetworkTables, which is where the pictures are read back from.
    TelemetryRegistry.registerBackend(
        "", new NetworkTablesTelemetryBackend(NetworkTableInstance.getDefault(), "/Telemetry"));
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setRobotMode(RobotMode.TELEOPERATED);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    Thread.sleep(100); // CTRE: give the simulated devices time to see the enable
  }

  /** Runs the scheduler, and so the simulation, for this long in real time. */
  private static void runFor(double seconds, Runnable everyLoop) throws InterruptedException {
    long end = System.nanoTime() + (long) (seconds * 1e9);
    while (System.nanoTime() < end) {
      DriverStationSim.notifyNewData();
      everyLoop.run();
      Scheduler.getDefault().run();
      Thread.sleep(20);
    }
  }

  /** A Talon on a CANcoder, configured like the bench arm but with test gains. */
  private static TalonFX arm(int motorId, CANcoder encoder, double kG, double kP, double kD) {
    // A non-zero magnet offset, to prove the simulated CANcoder still reads the true arm angle.
    encoder
        .getConfigurator()
        .apply(
            new CANcoderConfiguration()
                .withMagnetSensor(new MagnetSensorConfigs().withMagnetOffset(0.1)));

    var motor = new TalonFX(motorId, bus);
    motor
        .getConfigurator()
        .apply(
            new TalonFXConfiguration()
                .withMotorOutput(
                    new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive))
                .withSlot0(
                    new Slot0Configs()
                        .withKG(kG)
                        .withKP(kP)
                        .withKD(kD)
                        .withGravityType(GravityTypeValue.Arm_Cosine))
                .withMotionMagic(
                    new MotionMagicConfigs()
                        .withMotionMagicCruiseVelocity(1)
                        .withMotionMagicAcceleration(4))
                .withFeedback(
                    new FeedbackConfigs()
                        .withFeedbackRemoteSensorID(encoder.getDeviceID())
                        .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)));
    return motor;
  }

  /** The value a Mechanism2d published, found by its picture's name and the line's path. */
  private static double published(String picture, String path) {
    var inst = NetworkTableInstance.getDefault();
    for (var topic : inst.getTopics()) {
      String name = topic.getName();
      if (name.contains("/" + picture + "/") && name.endsWith(path)) {
        return inst.getEntry(name).getDouble(Double.NaN);
      }
    }
    var names = new StringBuilder();
    for (var topic : inst.getTopics()) {
      names.append('\n').append(topic.getName());
    }
    return fail("no topic for " + picture + " ending " + path + "; topics:" + names);
  }

  @Test
  void armReachesVerticalAndThePictureFollows() throws InterruptedException {
    var encoder = new CANcoder(42, bus);
    var motor = arm(41, encoder, 0.1, 8, 0.2);
    var sim = new ArmSim(motor, encoder);
    var request = new MotionMagicVoltage(0.25);
    MechanismViews.arm(
        "TestArm",
        () -> motor.getPosition().getValue(),
        () -> request.getPositionMeasure(),
        Arm.LENGTH);

    runFor(4.0, () -> motor.setControl(request));

    double truth = sim.getAngleRotations();
    System.out.printf(
        "arm truth %.4f  CANcoder %.4f  Talon %.4f%n",
        truth, encoder.getPosition().getValueAsDouble(), motor.getPosition().getValueAsDouble());
    assertEquals(0.25, truth, 0.01, "the simulated arm reaches vertical");
    assertEquals(truth, encoder.getPosition().getValueAsDouble(), 0.005, "CANcoder reads truth");
    assertEquals(truth, motor.getPosition().getValueAsDouble(), 0.005, "Talon reads truth");

    double armAngle = published("TestArm", "/pivot/arm/angle");
    double targetAngle = published("TestArm", "/pivot/target/angle");
    double targetLength = published("TestArm", "/pivot/target/length");
    System.out.printf(
        "picture: arm %.1f deg, target %.1f deg, target length %.3f m%n",
        armAngle, targetAngle, targetLength);
    assertEquals(90, armAngle, 2, "the picture's arm points straight up");
    assertEquals(90, targetAngle, 0.5, "the target line shows the Motion Magic goal");
    assertEquals(Arm.LENGTH.in(Meters), targetLength, 1e-6, "the target line is drawn");
  }

  /**
   * Holds the arm at horizontal (0.5 rotations, pointing left) on a weak kP, once with no kG and
   * once with the kG the physics actually needs. If the physics and Phoenix's Arm_Cosine agree on
   * which way is down, the right kG removes the sag. If they disagreed, it would double it.
   */
  @Test
  void gravityMatchesArmCosine() throws InterruptedException {
    // The voltage that holds the arm level: its weight at half its length, through the gearing.
    double torqueAtMotor = ArmSim.MASS_KG * 9.81 * ArmSim.LENGTH_METERS / 2 / ArmSim.GEARING;
    double kG = DCMotor.getKrakenX44(1).getVoltage(torqueAtMotor, 0);
    double kP = 20;

    var noKgEncoder = new CANcoder(45, bus);
    var withKgEncoder = new CANcoder(47, bus);
    var noKg = arm(44, noKgEncoder, 0, kP, 0.1);
    var withKg = arm(46, withKgEncoder, kG, kP, 0.1);
    var noKgSim = new ArmSim(noKg, noKgEncoder);
    var withKgSim = new ArmSim(withKg, withKgEncoder);

    var request = new MotionMagicVoltage(0.5);
    runFor(
        5.0,
        () -> {
          noKg.setControl(request);
          withKg.setControl(request);
        });

    double sagWithout = noKgSim.getAngleRotations() - 0.5;
    double sagWith = withKgSim.getAngleRotations() - 0.5;
    System.out.printf(
        "kG %.4f V: sag without %.4f rot, with %.4f rot (expected without %.4f)%n",
        kG, sagWithout, sagWith, kG / kP);
    assertEquals(kG / kP, sagWithout, 0.3 * kG / kP, "with no kG, gravity sags it by kG/kP");
    assertTrue(Math.abs(sagWith) < 0.3 * sagWithout, "the physics' kG takes most of the sag out");
  }

  @Test
  void flywheelSpinsUpAndTheGaugeFollows() throws InterruptedException {
    var motor = new TalonFX(43, bus);
    motor
        .getConfigurator()
        .apply(
            new TalonFXConfiguration()
                .withMotorOutput(
                    new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
                .withSlot0(new Slot0Configs().withKV(0.1).withKP(0.2))
                .withMotionMagic(new MotionMagicConfigs().withMotionMagicAcceleration(200)));

    var sim = new FlywheelSim(motor);
    var request = new MotionMagicVelocityVoltage(50);
    MechanismViews.flywheel(
        "TestFlywheel",
        () -> motor.getVelocity().getValue(),
        () -> request.getVelocityMeasure(),
        100);
    runFor(2.0, () -> motor.setControl(request));

    double truth = sim.getVelocityRotationsPerSecond();
    System.out.printf(
        "flywheel truth %.2f  Talon %.2f rps%n", truth, motor.getVelocity().getValueAsDouble());
    assertEquals(50, truth, 2.0, "the simulated wheel reaches 50 rps");
    assertEquals(truth, motor.getVelocity().getValueAsDouble(), 1.0, "Talon reads truth");

    double speedBar = published("TestFlywheel", "/speed/speed/length");
    double targetBar = published("TestFlywheel", "/target/target/length");
    System.out.printf("gauge: speed bar %.3f, target bar %.3f%n", speedBar, targetBar);
    assertEquals(truth / 100, speedBar, 0.02, "the speed bar is speed over full scale");
    assertEquals(0.5, targetBar, 0.01, "the target bar shows 50 of 100 rps");
  }
}
