package frc.robot.subsystems.flywheel;

import frc.robot.subsystems.flywheel.FlywheelConstants.FlywheelGains;
import org.littletonrobotics.junction.AutoLog;

/** The FlywheelIO interface is a hardware abstraction for a flywheel. */
public interface FlywheelIO {
  /**
   * The FlywheelIOInputs class contains inputs and current state of a flywheel.
   */
  @AutoLog
  public static class FlywheelIOInputs {
    public double velocity = 0.0;

    public double desiredVelocity = 0.0;

    public boolean[] motorsConnected = {false};

    public double[] motorPositions = {0.0};
    public double[] motorVelocities = {0.0};
    public double[] motorAccelerations = {0.0};

    public double[] motorVoltages = {0.0};
    public double[] motorCurrents = {0.0};
  }

  /**
   * Updates the flywheel inputs.
   *
   * @param inputs the current flywheel state
   */
  public default void updateInputs(FlywheelIOInputs inputs) {}

  /**
   * Sets the flywheel velocity setpoint.
   *
   * @param velocity the desired velocity in RPM
   */
  public default void setVelocity(double velocity) {}

  /**
   * Sets the flywheel voltage.
   *
   * @param voltage the desired voltage in volts
   */
  public default void setVoltage(double voltage) {}

  /**
   * Sets the flywheel PIDF gains.
   *
   * @param gains the desired PIDF gains
   */
  public default void setGains(FlywheelGains gains) {}

  public default String getName() {
    return "Flywheel";
  }
}
