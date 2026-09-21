package frc.robot.utils;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import org.wpilib.driverstation.DriverStationErrors;

/**
 * Helpers for TalonFX motors.
 *
 * <p>Phoenix's {@code apply(...)} sends a config to the motor once and returns a status code. It
 * does not retry on its own. {@link #applyConfigWithRetries} tries up to 5 times, which covers the
 * short CAN bus hiccups that can happen while the robot boots.
 */
public final class TalonFXUtil {

  private TalonFXUtil() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  /**
   * Applies a configuration to a TalonFX motor, retrying if it fails.
   *
   * @param motor The motor to configure
   * @param config The configuration to apply
   * @param maxRetries How many times to try
   * @return true if the config applied, false if every try failed
   */
  public static boolean applyConfigWithRetries(
      TalonFX motor, TalonFXConfiguration config, int maxRetries) {
    StatusCode status = StatusCode.OK;
    for (int i = 0; i < maxRetries; i++) {
      status = motor.getConfigurator().apply(config);
      if (status.isOK()) {
        return true;
      }
    }
    // Every try failed. Report it to the driver station so the problem is not silent.
    DriverStationErrors.reportError(
        "TalonFX "
            + motor.getDeviceID()
            + " failed to configure after "
            + maxRetries
            + " attempts ("
            + status
            + "). Check CAN wiring and device ID.",
        false);
    return false;
  }

  /**
   * Applies a configuration to a TalonFX motor, trying up to 5 times.
   *
   * @param motor The motor to configure
   * @param config The configuration to apply
   * @return true if the config applied, false if every try failed
   */
  public static boolean applyConfigWithRetries(TalonFX motor, TalonFXConfiguration config) {
    return applyConfigWithRetries(motor, config, 5);
  }
}
