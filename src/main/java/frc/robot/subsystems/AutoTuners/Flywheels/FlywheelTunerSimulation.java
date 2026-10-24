package frc.robot.subsystems.AutoTuners.Flywheels;


import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/** Portable integration checks using the production tuning algorithm. */
public final class FlywheelTunerSimulation {
    private static int checks;
    private static void check(boolean value, String message) {
        checks++;
        if (!value) throw new AssertionError(message);
    }
    private static void enable(boolean value) {
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setTest(true);
        DriverStationSim.setEnabled(value);
        DriverStationSim.notifyNewData();
    }
    private static void tick(FlywheelAutoTuner tuner, FlywheelAutoTunerIOSim io, double maxVolts) {
        io.update(0.02);
        SimHooks.stepTiming(0.02);
        tuner.periodic();
        check(Double.isFinite(io.getCommandVolts()) && Math.abs(io.getCommandVolts()) <= maxVolts, "Voltage bound");
    }
    public static void main(String[] args) {
        check(HAL.initialize(500, 0), "HAL initialize");
        SimHooks.pauseTiming();
        double[] inertias = {0.003, 0.01, 0.001};
        for (double inertia : inertias) {
            var io = new FlywheelAutoTunerIOSim(DCMotor.getFalcon500(1), inertia, 1, 12);
            // This ideal voltage-driven plant has no controller current limiter.
            var config = inertia == 0.01 ? new FlywheelAutoTuner.Config(65, 100, 7, 1.5, 3.5, 5)
                : FlywheelAutoTuner.Config.defaults();
            var tuner = new FlywheelAutoTuner(io, config, "FlywheelSimulation/" + inertia);
            enable(false);
            tuner.beginTuning();
            check(tuner.getStatus().contains("requires Test"), "Test mode required");
            enable(true);
            tuner.runCandidate(20);
            check(!tuner.isRunning(), "Candidate required");
            io.setVoltage(1);
            tuner.beginTuning();
            check(io.getCommandVolts() == 0, "Calibration immediately clears previous output");
            for (int i = 0; i < 255; i++) tick(tuner, io, config.maxRunVolts());
            System.out.printf("IDENTIFY J=%.3f status=%s samples=%d gains=%s%n",
                inertia, tuner.getStatus(), tuner.getSampleCount(), tuner.getCandidate());
            check(tuner.getCandidate() != null, "Identification: " + tuner.getStatus());
            for (double invalid : new double[] {Double.NaN, Double.POSITIVE_INFINITY, -1, config.maxTestRPS()+1}) {
                tuner.runCandidate(invalid);
                check(!tuner.isRunning() && io.getCommandVolts() == 0, "Invalid target stops motor");
            }
            tuner.runCandidate(20);
            double peak = 0, finalError = 0;
            for (int i = 0; i < 250; i++) {
                tick(tuner, io, config.maxRunVolts());
                peak = Math.max(peak, io.readInputs().velocityRPS());
                if (i >= 200) finalError = Math.max(finalError, Math.abs(io.readInputs().velocityRPS()-20));
            }
            System.out.printf("RUN J=%.3f finalRPS=%.6f peakRPS=%.6f finalSecondMaxError=%.6f%n",
                inertia, io.readInputs().velocityRPS(), peak, finalError);
            check(tuner.isRunning() && finalError < 0.2, "20 RPS tracking");
            io.setVelocityRPS(io.readInputs().velocityRPS() * 0.7);
            for (int i = 0; i < 150; i++) tick(tuner, io, config.maxRunVolts());
            double recovery = Math.abs(io.readInputs().velocityRPS()-20);
            System.out.printf("RECOVERY J=%.3f errorAfter3s=%.6f%n", inertia, recovery);
            check(recovery < 0.2, "Disturbance recovery");
            enable(false);
            tuner.runCandidate(20);
            check(!tuner.isRunning() && io.getCommandVolts() == 0, "Disabled candidate request immediately stops output");
            enable(true);
            tuner.runCandidate(20);
            tick(tuner, io, config.maxRunVolts());
            enable(false);
            tuner.periodic();
            check(!tuner.isRunning() && io.getCommandVolts() == 0, "Disable stops output");
            enable(true);
            tuner.runCandidate(20);
            io.setSensorOverride(new FlywheelAutoTuner.Inputs(config.maxTestRPS()+5, 0, 0, true));
            tuner.runCandidate(20);
            check(tuner.getStatus().startsWith("ABORTED") && io.getCommandVolts() == 0
                && tuner.getCandidate() == null && !tuner.isRunning(), "Unsafe candidate request immediately aborts and clears gains");
            for (FlywheelAutoTuner.Inputs fault : new FlywheelAutoTuner.Inputs[] {
                    new FlywheelAutoTuner.Inputs(config.maxTestRPS()+5, 0, 0, true),
                    new FlywheelAutoTuner.Inputs(0, 0, config.maxCurrentAmps()+3, true),
                    new FlywheelAutoTuner.Inputs(Double.NaN, 0, 0, true),
                    new FlywheelAutoTuner.Inputs(0, Double.NaN, 0, true),
                    new FlywheelAutoTuner.Inputs(0, 0, 0, false)}) {
                io.setSensorOverride(null);
                io.setVelocityRPS(0);
                tuner.beginTuning();
                io.setSensorOverride(fault);
                tuner.periodic();
                check(tuner.getStatus().startsWith("ABORTED") && io.getCommandVolts() == 0, "Sensor/current fault abort");
            }
            tuner.stop();
            CommandScheduler.getInstance().unregisterSubsystem(tuner);
        }
        System.out.printf("RESULT PASS checks=%d cases=%d%n", checks, inertias.length);
        System.exit(0);
    }
}

