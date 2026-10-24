package frc.robot.subsystems.AutoTuners.Arms;


import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/** Repeatable integration checks of the production tuner against nonlinear arm physics. */
public final class ArmTunerSimulation {
    private static int checks;
    static void check(boolean condition, String message) {
        checks++;
        if (!condition) throw new AssertionError(message);
    }
    static void enable(boolean value) {
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setTest(true);
        DriverStationSim.setEnabled(value);
        DriverStationSim.notifyNewData();
    }
    static void tick(ArmAutoTuner tuner, ArmAutoTunerIOSim io) {
        io.update(0.02);
        SimHooks.stepTiming(0.02);
        tuner.periodic();
        check(Double.isFinite(io.getCommandVolts()) && Math.abs(io.getCommandVolts()) <= 6, "Voltage bound");
    }
    static ArmAutoTuner.Config config() {
        return new ArmAutoTuner.Config(-1.1, 1.5, 0.08, 4, 60, 6, 0.4, 12, 1, 3);
    }
    public static void main(String[] args) {
        check(HAL.initialize(500, 0), "HAL initialize");
        SimHooks.pauseTiming();
        double[][] cases = {
            {0.2, 10, 0.5, 0, 0, 0}, {0.5, 30, 0.65, 0, 0, 0},
            {0.02, 60, 0.35, 0, 0, 0}, {0.2, 10, 0.5, 0.15, 0, 0},
            {0.2, 10, 0.5, 0.15, 0.001, 0.01}};
        for (double[] c : cases) {
            ArmAutoTunerIOSim io = new ArmAutoTunerIOSim(c[0], c[1], c[2], -1.1, 1.5, 0.2, c[3]);
            io.setSensorNoise(c[4], c[5]);
            ArmAutoTuner tuner = new ArmAutoTuner(io, config());
            enable(false);
            tuner.beginTuning();
            check(tuner.getStatus().contains("requires Test"), "Disabled calibration rejected");
            enable(true);
            tuner.runCandidate(0.2);
            check(!tuner.isRunning(), "Candidate required");
            tuner.beginTuning();
            for (int i = 0; i < 601; i++) tick(tuner, io);
            System.out.printf("IDENTIFY J=%.3f gear=%.0f friction=%.2f angleNoise=%.3f velocityNoise=%.3f status=%s gains=%s%n",
                c[0], c[1], c[3], c[4], c[5], tuner.getStatus(), tuner.getCandidate());
            check(tuner.getCandidate() != null, "Identification: " + tuner.getStatus());
            DCMotor motor = DCMotor.getNEO(1);
            double expectedKV = c[1] / motor.KvRadPerSecPerVolt;
            double expectedKA = c[0] * motor.rOhms / (c[1] * motor.KtNMPerAmp);
            double expectedKG = (3*c[0]/(c[2]*c[2])) * 9.8*c[2]/2 * motor.rOhms/(c[1]*motor.KtNMPerAmp);
            var gains = tuner.getCandidate();
            check(Math.abs(gains.kV()/expectedKV - 1) < 0.15, "kV accuracy");
            check(Math.abs(gains.kA()/expectedKA - 1) < 0.25, "kA accuracy");
            check(Math.abs(gains.kG()/expectedKG - 1) < 0.15, "kG accuracy");
            check(Math.abs(gains.kS() - c[3]) < 0.06, "kS accuracy");
            for (double target : new double[] {0.7, -0.3, 0.95}) {
                tuner.runCandidate(target);
                double maximumFinalError = 0;
                for (int i = 0; i < 250; i++) {
                    tick(tuner, io);
                    if (i >= 200) maximumFinalError = Math.max(maximumFinalError, Math.abs(io.readInputs().angleRad() - target));
                }
                System.out.printf("POSITION gear=%.0f target=%.2f finalErrorDeg=%.4f status=%s%n",
                    c[1], target, Math.toDegrees(maximumFinalError), tuner.getStatus());
                check(tuner.isRunning() && maximumFinalError < Math.toRadians(2), "Position tracking");
            }
            io.setState(0.8, 0);
            for (int i = 0; i < 150; i++) tick(tuner, io);
            double recovery = Math.abs(io.readInputs().angleRad() - 0.95);
            System.out.printf("RECOVERY gear=%.0f errorDeg=%.4f%n", c[1], Math.toDegrees(recovery));
            check(recovery < Math.toRadians(2), "Disturbance recovery");
            for (double invalid : new double[] {Double.NaN, Double.POSITIVE_INFINITY, -1.1, 1.5}) {
                tuner.runCandidate(invalid);
                check(!tuner.isRunning() && io.getCommandVolts() == 0, "Invalid target stops motor");
            }
            tuner.runCandidate(0.5);
            enable(false);
            tuner.periodic();
            check(!tuner.isRunning() && io.getCommandVolts() == 0, "Disable stops motor");
            enable(true);
            ArmAutoTuner.Inputs[] faults = {
                new ArmAutoTuner.Inputs(1.45, 0, 0, 0, true),
                new ArmAutoTuner.Inputs(-1.05, 0, 0, 0, true),
                new ArmAutoTuner.Inputs(0.2, 5, 0, 0, true),
                new ArmAutoTuner.Inputs(0.2, 0, 0, 61, true),
                new ArmAutoTuner.Inputs(Double.NaN, 0, 0, 0, true),
                new ArmAutoTuner.Inputs(0.2, Double.NaN, 0, 0, true),
                new ArmAutoTuner.Inputs(0.2, 0, Double.NaN, 0, true),
                new ArmAutoTuner.Inputs(0.2, 0, 0, Double.NaN, true),
                new ArmAutoTuner.Inputs(0.2, 0, 0, 0, false)};
            for (ArmAutoTuner.Inputs fault : faults) {
                io.setSensorOverride(null);
                io.setState(0.2, 0);
                tuner.beginTuning();
                tick(tuner, io);
                check(Math.abs(io.getCommandVolts()) > 0, "Identification exercising motor before fault");
                io.setSensorOverride(fault);
                tuner.periodic();
                check(tuner.getStatus().startsWith("ABORTED") && io.getCommandVolts() == 0
                    && tuner.getCandidate() == null, "Active fault aborts and clears candidate");
                tuner.beginTuning();
                check(tuner.getStatus().startsWith("ABORTED") && io.getCommandVolts() == 0, "Fault prevents tuning");
            }
            io.setSensorOverride(null);
            io.setState(0.2, 0.2);
            tuner.beginTuning();
            check(tuner.getStatus().contains("stopped first"), "Moving arm rejected");
            io.setState(-0.8, 0);
            tuner.beginTuning();
            check(tuner.getStatus().contains("Not enough travel"), "Sweep clearance required");
            io.setState(0.2, 0);
            tuner.beginTuning();
            SimHooks.stepTiming(0.05);
            tuner.periodic();
            check(tuner.getStatus().contains("20 ms") && io.getCommandVolts() == 0, "Timing failure aborts");
            tuner.stop();
            CommandScheduler.getInstance().unregisterSubsystem(tuner);
        }
        final double[] jammedVolts = {0};
        ArmAutoTuner.IO jammed = new ArmAutoTuner.IO() {
            public ArmAutoTuner.Inputs readInputs() {
                return new ArmAutoTuner.Inputs(0.2, 0, jammedVolts[0], 0, true);
            }
            public void setVoltage(double volts) { jammedVolts[0] = volts; }
        };
        ArmAutoTuner stalledTuner = new ArmAutoTuner(jammed, config());
        stalledTuner.beginTuning();
        for (int i = 0; i < 601; i++) {
            SimHooks.stepTiming(0.02);
            stalledTuner.periodic();
        }
        check(stalledTuner.getCandidate() == null && jammedVolts[0] == 0
            && stalledTuner.getStatus().startsWith("Identification failed"), "Stalled arm rejects model");
        System.out.printf("STALLED status=%s%n", stalledTuner.getStatus());
        CommandScheduler.getInstance().unregisterSubsystem(stalledTuner);
        System.out.printf("RESULT PASS checks=%d cases=%d plus stalled arm%n", checks, cases.length);
        System.exit(0);
    }
}

