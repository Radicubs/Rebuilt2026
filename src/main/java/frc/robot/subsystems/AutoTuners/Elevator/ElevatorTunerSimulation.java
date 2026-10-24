package frc.robot.subsystems.AutoTuners.Elevator;


import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/** Integration checks of the portable production tuner against elevator physics. */
public final class ElevatorTunerSimulation {
    private static int checks;
    private static void check(boolean condition, String message) {
        checks++;
        if (!condition) throw new AssertionError(message);
    }
    private static void enable(boolean value) {
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setTest(true);
        DriverStationSim.setEnabled(value);
        DriverStationSim.notifyNewData();
    }
    private static void tick(ElevatorAutoTuner tuner, ElevatorAutoTunerIOSim io) {
        io.update(0.02);
        SimHooks.stepTiming(0.02);
        tuner.periodic();
        check(Double.isFinite(io.getCommandVolts()) && Math.abs(io.getCommandVolts()) <= 6, "Voltage bound");
    }
    private static ElevatorAutoTuner.Config config() {
        return new ElevatorAutoTuner.Config(0, 1.5, 0.05, 1.5, 80, 6, 0.2, 30, 2, 3);
    }
    private static ElevatorAutoTuner.Inputs sensor(double height, double speed, double volts,
            double amps, boolean connected, boolean homed, boolean lower, boolean upper) {
        return new ElevatorAutoTuner.Inputs(height, speed, volts, amps, connected, homed, lower, upper);
    }
    public static void main(String[] args) {
        check(HAL.initialize(500, 0), "HAL initialize");
        SimHooks.pauseTiming();
        double[][] cases = {
            {4, 5, 0.025, 0, 0, 0}, {10, 10, 0.03, 0, 0, 0}, {15, 20, 0.04, 0, 0, 0},
            {10, 10, 0.03, 0.12, 0, 0}, {10, 10, 0.03, 0.12, 0.0005, 0.005}};
        for (int index = 0; index < cases.length; index++) {
            double[] c = cases[index];
            DCMotor motor = DCMotor.getNEO(1);
            var io = new ElevatorAutoTunerIOSim(motor, c[0], c[1], c[2], 0, 1.5, 0.75, c[3]);
            io.setSensorNoise(c[4], c[5]);
            var tuner = new ElevatorAutoTuner(io, config(), "ElevatorSimulation/" + index);
            enable(false);
            tuner.beginTuning();
            check(tuner.getStatus().contains("requires Test"), "Disabled calibration rejected");
            enable(true);
            tuner.runCandidate(0.8);
            check(!tuner.isRunning(), "Candidate required");
            tuner.beginTuning();
            for (int i = 0; i < 601; i++) tick(tuner, io);
            System.out.printf("IDENTIFY mass=%.1f gear=%.0f radius=%.3f friction=%.2f noise=%.4f status=%s samples=%d gains=%s%n",
                c[0], c[1], c[2], c[3], c[4], tuner.getStatus(), tuner.getSampleCount(), tuner.getCandidate());
            check(tuner.getCandidate() != null, "Identification: " + tuner.getStatus());
            double expectedKV = c[1] / (c[2] * motor.KvRadPerSecPerVolt);
            double expectedKA = c[0] * motor.rOhms * c[2] / (c[1] * motor.KtNMPerAmp);
            var gains = tuner.getCandidate();
            check(Math.abs(gains.kV()/expectedKV-1) < 0.15, "kV accuracy");
            check(Math.abs(gains.kA()/expectedKA-1) < 0.25, "kA accuracy");
            check(Math.abs(gains.kG()/(9.8*expectedKA)-1) < 0.15, "kG accuracy");
            check(Math.abs(gains.kS()-c[3]) < 0.06, "kS accuracy");
            for (double target : new double[] {0.95, 0.4, 1.15}) {
                tuner.runCandidate(target);
                double finalError = 0;
                for (int i = 0; i < 250; i++) {
                    tick(tuner, io);
                    if (i >= 200) finalError = Math.max(finalError, Math.abs(io.readInputs().heightMeters()-target));
                }
                System.out.printf("POSITION case=%d target=%.2f finalSecondMaxErrorMm=%.4f status=%s%n",
                    index, target, finalError*1000, tuner.getStatus());
                check(tuner.isRunning() && finalError < 0.01, "Height tracking within 10 mm");
            }
            io.setState(1.05, 0);
            for (int i = 0; i < 150; i++) tick(tuner, io);
            double recovery = Math.abs(io.readInputs().heightMeters()-1.15);
            System.out.printf("RECOVERY case=%d errorAfter3sMm=%.4f%n", index, recovery*1000);
            check(recovery < 0.01, "Disturbance recovery within 10 mm");
            for (double invalid : new double[] {Double.NaN, Double.POSITIVE_INFINITY, 0, 1.5}) {
                tuner.runCandidate(invalid);
                check(!tuner.isRunning() && io.getCommandVolts() == 0, "Invalid target stops output");
            }
            tuner.runCandidate(0.8);
            enable(false);
            tuner.periodic();
            check(!tuner.isRunning() && io.getCommandVolts() == 0, "Disable stops output");
            enable(true);
            ElevatorAutoTuner.Inputs[] faults = {
                sensor(0.02,0,0,0,true,true,false,false), sensor(1.48,0,0,0,true,true,false,false),
                sensor(0.75,2,0,0,true,true,false,false), sensor(0.75,0,0,81,true,true,false,false),
                sensor(Double.NaN,0,0,0,true,true,false,false), sensor(0.75,Double.NaN,0,0,true,true,false,false),
                sensor(0.75,0,Double.NaN,0,true,true,false,false), sensor(0.75,0,0,Double.NaN,true,true,false,false),
                sensor(0.75,0,0,0,false,true,false,false), sensor(0.75,0,0,0,true,false,false,false),
                sensor(0.75,0,0,0,true,true,true,false), sensor(0.75,0,0,0,true,true,false,true)};
            // Also verify a fault while candidate position control is actively applying voltage.
            tuner.runCandidate(0.8);
            io.setSensorOverride(faults[11]);
            tuner.periodic();
            check(!tuner.isRunning() && tuner.getCandidate() == null && io.getCommandVolts() == 0,
                "Limit switch aborts candidate control and clears gains");
            for (var fault : faults) {
                io.setSensorOverride(null);
                io.setState(0.75, 0);
                tuner.beginTuning();
                tick(tuner, io);
                check(Math.abs(io.getCommandVolts()) > 0, "Active identification before fault");
                io.setSensorOverride(fault);
                tuner.periodic();
                check(tuner.getStatus().startsWith("ABORTED") && io.getCommandVolts() == 0
                    && tuner.getCandidate() == null, "Active fault abort");
                tuner.beginTuning();
                check(tuner.getStatus().startsWith("ABORTED"), "Unsafe restart rejected");
            }
            io.setSensorOverride(null);
            io.setState(0.75, 0.2);
            tuner.beginTuning();
            check(tuner.getStatus().contains("stopped first"), "Moving elevator rejected");
            io.setState(0.15, 0);
            tuner.beginTuning();
            check(tuner.getStatus().contains("Not enough travel"), "Sweep clearance required");
            io.setState(0.75, 0);
            tuner.beginTuning();
            SimHooks.stepTiming(0.05);
            tuner.periodic();
            check(tuner.getStatus().contains("20 ms") && io.getCommandVolts() == 0, "Timing fault abort");
            tuner.stop();
            CommandScheduler.getInstance().unregisterSubsystem(tuner);
        }
        final double[] jammedVolts = {0};
        var jammedIO = new ElevatorAutoTuner.IO() {
            public ElevatorAutoTuner.Inputs readInputs() {
                return sensor(0.75, 0, jammedVolts[0], 0, true, true, false, false);
            }
            public void setVoltage(double volts) { jammedVolts[0] = volts; }
        };
        var stalled = new ElevatorAutoTuner(jammedIO, config());
        stalled.beginTuning();
        for (int i = 0; i < 601; i++) { SimHooks.stepTiming(0.02); stalled.periodic(); }
        check(stalled.getCandidate() == null && jammedVolts[0] == 0
            && stalled.getStatus().startsWith("Identification failed"), "Stalled elevator rejects model");
        System.out.printf("STALLED status=%s%n", stalled.getStatus());
        CommandScheduler.getInstance().unregisterSubsystem(stalled);
        System.out.printf("RESULT PASS checks=%d cases=%d plus stalled elevator%n", checks, cases.length);
        System.exit(0);
    }
}


