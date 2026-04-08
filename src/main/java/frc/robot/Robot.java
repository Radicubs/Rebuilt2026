// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.*;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.opencv.photo.Photo;


public class Robot extends LoggedRobot
{
    private Command autonomousCommand;

    private final RobotContainer robotContainer;
    
    
    public Robot()
    {
        Logger.recordMetadata("Rebuilt2026", "Rebuilt"); // Set a metadata value

        if (isReal()) {
            Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        } else {
            setUseTiming(false); // Run as fast as possible
            String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
            Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
            Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
        }

        Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.
        robotContainer = new RobotContainer();
    }
    
    
    @Override
    public void robotPeriodic()
    {
        long currentTime = NetworkTablesJNI.now();

        CommandScheduler.getInstance().run();

        SmartDashboard.putNumber("CAN BUS Utilization", CANBus.roboRIO().getStatus().BusUtilization);
        SmartDashboard.putNumber("CAN BUS REC", CANBus.roboRIO().getStatus().REC);
        SmartDashboard.putNumber("CAN BUS TEC", CANBus.roboRIO().getStatus().TEC);

        SmartDashboard.putNumber("CAN BUS TxCount", CANBus.roboRIO().getStatus().TxFullCount);

        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());

        double velX = Swerve.getInstance().getRobotRelativeSpeeds().vxMetersPerSecond;
        double velY = Swerve.getInstance().getRobotRelativeSpeeds().vyMetersPerSecond;
        double photonTime = 0;
        Pose2d photonPose = PhotonVision.getInstance().getRobotFieldPose();
        Pose2d futurePose = null;

        if(PhotonVision.getInstance().hasMultiTag()) {
            photonTime = PhotonVision.getInstance().getTimeStamp();
            futurePose = new Pose2d(photonPose.getX() + velX*(Math.abs(photonTime - currentTime)), photonPose.getY() + velY*(Math.abs(photonTime - (currentTime))), photonPose.getRotation());
//          Swerve.getInstance().setPose(PhotonVision.getInstance().getRobotFieldPose());
            Swerve.getInstance().setPose(futurePose);
        }
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
        PhotonVision.getInstance();
        
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
        Shooter.getInstance().stop();
        Transfer.getInstance().setTransferSpeed(0);
        Pivot.getInstance().cancelPID();
        Intake.getInstance().setIntakeSpeed(0);
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
