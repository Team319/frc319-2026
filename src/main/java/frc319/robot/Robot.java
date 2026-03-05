// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc319.robot;

import java.lang.reflect.Field;
import java.util.Optional;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc319.lib.util.FieldUtils;

public class Robot extends LoggedRobot {
  
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  public boolean hasBeenEnabled = false;

  public String dynamicAutoInput = "";

  double manualClimbSetpoint = 0.0;

  @Override
  public void robotInit() {
    //=============================================
    // START : Required setup for AdvantageKit Logging
    //=============================================
    
    Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);

    if (isReal()) {
        Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        //new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging  
    } else if(isSimulation()){
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    }
    else{
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
        Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    }
    
    // Logger.disableDeterministicTimestamps() // See "Deterministic Timestamps" in the "Understanding Data Flow" page
    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.
    
    //=============================================
    // END : Required setup for AdvantageKit Logging
    //=============================================

    CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());

    SmartDashboard.putString("DynamicAutoInput", "");

    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    logStates();

    if(!hasBeenEnabled){
      SmartDashboard.putString("Dynamic Reading", m_robotContainer.dynamicAutoInput);
    }
  }

  @Override
  public void disabledInit() {
    m_robotContainer.driverController.setRumble(RumbleType.kBothRumble, 0.0);
   }
  

  @Override
  public void disabledPeriodic() {

    // Read dynamic auto selection from SmartDashboard
    m_robotContainer.dynamicAutoInput = SmartDashboard.getString("DynamicAutoInput", "");

    //System.out.println("DynamicAuto Input :" +m_robotContainer.dynamicAutoInput);

   if (!hasBeenEnabled) {
    Optional<Alliance> allianceColor = DriverStation.getAlliance();
    allianceColor.ifPresent(alliance -> {
      if (alliance == Alliance.Red) {
        m_robotContainer.drive.setHeading(0.0);
        //System.out.println("Red Alliance: Setting heading to 0 degrees.");
      } else if (alliance == Alliance.Blue) {
        m_robotContainer.drive.setHeading(180.0);
        //System.out.println("Blue Alliance: Setting heading to 180 degrees.");
      }
    });

     if(!allianceColor.isPresent()){
       System.out.println("Alliance color is not set yet.");
     }
   }

  
  }

  @Override
  public void disabledExit() {
    hasBeenEnabled = true;

  }

  @Override
  public void autonomousInit() {
    
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

     if (m_autonomousCommand != null) {
        CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }


  }

  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {

    // TODO add Trench / Autopilot rumble feedback

    // if(m_robotContainer.drive.nearTheReef){
    //   m_robotContainer.driverController.setRumble(RumbleType.kBothRumble, 0.1);
    // } else {
    //   m_robotContainer.driverController.setRumble(RumbleType.kBothRumble, 0.0);
    // }




    // TODO - for manual climb control (may not need)

    // ===============  Manual climbing ===============================================
    // if(m_robotContainer.superstructure.climber.getClimbMode()){
    //   m_robotContainer.operatorController.setRumble(RumbleType.kBothRumble, 0.1);

    //   if( Math.abs( m_robotContainer.operatorController.getLeftY()) >= 0.1 )
    //   {
    //     manualClimbSetpoint = m_robotContainer.superstructure.climber.getPosition() + m_robotContainer.operatorController.getLeftY()*5.0;
    //     m_robotContainer.superstructure.climber.runPosition(manualClimbSetpoint);
    //   }
    // }
    // else{
    //   m_robotContainer.operatorController.setRumble(RumbleType.kBothRumble, 0.0);
    // }

    // ===============  elevator nudge ===============================================

    




  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
  
  private void logStates() {
  boolean isInAllianceZone = FieldUtils.isInAllianceZone(m_robotContainer.drive.getGlobalPose().getTranslation().toTranslation2d());
  boolean isInNeutralZone = FieldUtils.isInNeutralZone(m_robotContainer.drive.getGlobalPose().getTranslation().toTranslation2d());
  boolean isInOpposingZone = FieldUtils.isInOpposingZone(m_robotContainer.drive.getGlobalPose().getTranslation().toTranslation2d());

  boolean isInAnyTrenchOpening = FieldUtils.isInAnyTrenchOpening(m_robotContainer.drive.getGlobalPose().getTranslation().toTranslation2d());

  boolean isLeftSideOfField = FieldUtils.isLeftSide(m_robotContainer.drive.getGlobalPose().getTranslation().toTranslation2d());
  boolean isRightSideOfField = FieldUtils.isRightSide(m_robotContainer.drive.getGlobalPose().getTranslation().toTranslation2d());
  
  Logger.recordOutput("FieldStates/isInAllianceZone", isInAllianceZone);
  Logger.recordOutput("FieldStates/isInNeutralZone", isInNeutralZone);
  Logger.recordOutput("FieldStates/isInOpposingZone", isInOpposingZone);
  Logger.recordOutput("FieldStates/isInAnyTrenchOpening", isInAnyTrenchOpening);

  Logger.recordOutput("FieldStates/isLeftSideOfField", isLeftSideOfField);
  Logger.recordOutput("FieldStates/isRightSideOfField", isRightSideOfField);

  }
}


