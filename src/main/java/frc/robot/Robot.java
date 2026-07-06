// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.transfer.Transfer;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;


public class Robot extends LoggedRobot
{
    // set true to deterministically re-run a log instead of physics sim
    private static final boolean REPLAY = false;

    private Command autonomousCommand;

    private final RobotContainer robotContainer;


    public Robot()
    {
        Logger.recordMetadata("Rebuilt2026", "Rebuilt");
        Logger.recordMetadata("RobotMode", isReal() ? "REAL" : (REPLAY ? "REPLAY" : "SIM"));

        if (isReal()) {
            Logger.addDataReceiver(new WPILOGWriter()); // USB /U/logs if present, else onboard
            Logger.addDataReceiver(new NT4Publisher());
        } else if (REPLAY) {
            setUseTiming(false);
            String logPath = LogFileUtil.findReplayLog();
            Logger.setReplaySource(new WPILOGReader(logPath));
            Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        } else {
            Logger.addDataReceiver(new NT4Publisher());
        }

        Logger.start();
        robotContainer = new RobotContainer();
    }
    
    
    @Override
    public void robotPeriodic()
    {

        CommandScheduler.getInstance().run();

        logOutputs();

        SmartDashboard.putNumber("CAN BUS Utilization", CANBus.roboRIO().getStatus().BusUtilization);
        SmartDashboard.putNumber("CAN BUS REC", CANBus.roboRIO().getStatus().REC);
        SmartDashboard.putNumber("CAN BUS TEC", CANBus.roboRIO().getStatus().TEC);

        SmartDashboard.putNumber("CAN BUS TxCount", CANBus.roboRIO().getStatus().TxFullCount);

        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());

    }

    private void logOutputs()
    {
        Logger.recordOutput("Auto/Command", autonomousCommand != null ? autonomousCommand.getName() : "none");
    }


    @Override
    public void disabledInit() {

    }
    
    
    @Override
    public void disabledPeriodic() {}
    
    
    @Override
    public void disabledExit() {}
    
    
    @Override
    public void autonomousInit(){
        autonomousCommand = robotContainer.getAutonomousCommand();
        Vision.getInstance();
        
        if (autonomousCommand != null)
        {
            CommandScheduler.getInstance().schedule(autonomousCommand);
        }
    }
    
    
    @Override
    public void autonomousPeriodic() {
    }
    
    
    @Override
    public void autonomousExit() {
        Shooter.getInstance().setShooterSpeeds(0,0,0);
        Transfer.getInstance().setTransferSpeed(0);
        Pivot.getInstance().cancelPID();
        Intake.getInstance().setVelocity(0);
    }
    
    
    @Override
    public void teleopInit()
    {
        if (autonomousCommand != null)
        {
            autonomousCommand.cancel();
        }
    }
    
    
    @Override
    public void teleopPeriodic() {
    }
    
    
    @Override
    public void teleopExit() {
        Pivot.getInstance().cancelPID();
    }
    
    
    @Override
    public void testInit()
    {
        CommandScheduler.getInstance().cancelAll();
    }
    
    
    @Override
    public void testPeriodic() {}
    
    
    @Override
    public void testExit() {}
}
