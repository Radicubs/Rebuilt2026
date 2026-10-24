package frc.robot.subsystems.AutoTuners.Flywheels;

import java.util.Objects;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;

/** Optional Phoenix 6 adapter. Reuses a caller-owned TalonFX; never chooses a CAN ID.
 * Copy this file only for Phoenix projects. Other controllers can implement the core IO directly.
 */
public final class FlywheelAutoTunerTalonFXIO implements FlywheelAutoTuner.IO {
    private final TalonFX motor;
    private final double motorRotationsPerFlywheelRotation;
    public FlywheelAutoTunerTalonFXIO(TalonFX motor, double motorRotationsPerFlywheelRotation,
                                    double supplyCurrentLimitAmps) {
        this.motor = Objects.requireNonNull(motor);
        if (!Double.isFinite(motorRotationsPerFlywheelRotation) || motorRotationsPerFlywheelRotation <= 0
                || !Double.isFinite(supplyCurrentLimitAmps) || supplyCurrentLimitAmps <= 0)
            throw new IllegalArgumentException("Positive gearing and current limit required");
        this.motorRotationsPerFlywheelRotation = motorRotationsPerFlywheelRotation;
        CurrentLimitsConfigs limits = new CurrentLimitsConfigs();
        limits.SupplyCurrentLimitEnable = true;
        limits.SupplyCurrentLimit = supplyCurrentLimitAmps;
        if (!motor.getConfigurator().apply(limits).isOK())
            throw new IllegalStateException("Flywheel current limit configuration failed");
    }
    @Override public FlywheelAutoTuner.Inputs readInputs() {
        var velocity = motor.getRotorVelocity();
        var voltage = motor.getMotorVoltage();
        var current = motor.getSupplyCurrent();
        return new FlywheelAutoTuner.Inputs(velocity.getValueAsDouble() / motorRotationsPerFlywheelRotation,
            voltage.getValueAsDouble(), current.getValueAsDouble(),
            velocity.getStatus().isOK() && voltage.getStatus().isOK() && current.getStatus().isOK());
    }
    @Override public void setVoltage(double volts) { motor.setVoltage(volts); }
}

