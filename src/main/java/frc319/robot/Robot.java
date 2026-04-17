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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc319.lib.util.FieldUtils;
import frc319.robot.RobotContainer.RobotStates;
import frc319.robot.subsystems.intake.IntakeConstants.IntakeStates;
import frc319.robot.subsystems.serializer.SerializerConstants.SerializerStates;

public class Robot extends LoggedRobot {
  
  private Command m_autonomousCommand;

  public static RobotContainer m_robotContainer;

  public boolean hasBeenEnabled = false;

  public String dynamicAutoInput = "";

  double manualClimbSetpoint = 0.0;

  String gameData;

  @Override
  public void robotInit() {
    //=============================================
    // START : Required setup for AdvantageKit Logging
    //=============================================
    
    Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);

    if (isReal()) {
        //Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        //new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging  
    } else if(isSimulation()){
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    }
    else{ // Replay
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

    SmartDashboard.putString("DynamicAutoInput", "h2x5o1s9");

    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    Logger.recordOutput("Debug/flywheel/isAtTarget", m_robotContainer.flywheels.isAtTargetVelocity());
    Logger.recordOutput("Debug/turret/isAtTarget", m_robotContainer.turret.isAtTargetPosition());
    Logger.recordOutput("Debug/hood/isAtTarget", m_robotContainer.hood.isAtTargetPosition());

    switch(m_robotContainer.getCurrentRobotState()) {

      case SHOOTING:
      case SHOOTING_AUTO:
      case SHOOTING_DUMB:
      case SHOOTING_ON_MOVE:
      case SNOWBLOW:

        // if(m_robotContainer.startedFiringTime > 0.0 && (Timer.getFPGATimestamp() - m_robotContainer.startedFiringTime) > 2.5){
        //   m_robotContainer.intakeExtension.setState(IntakeStates.JOSTLE);
        // }

        // if(m_robotContainer.startedFiringTime > 0.0 && (Timer.getFPGATimestamp() - m_robotContainer.startedFiringTime) > 3.0){
        //   m_robotContainer.intakeExtension.setState(IntakeStates.RETRACTED);
        //   m_robotContainer.startedFiringTime = Timer.getFPGATimestamp();
        // }

        if(m_robotContainer.flywheels.isAtTargetVelocity() 
            && m_robotContainer.turret.isAtTargetPosition()
           // && m_robotContainer.turret.isAtTargetVelocity()
            && m_robotContainer.hood.isAtTargetPosition()){
              m_robotContainer.ballTunnel.setState(SerializerStates.SHOOT);
              m_robotContainer.spindexer.setState(SerializerStates.SHOOT);
            }
        else{
              // m_robotContainer.ballTunnel.setState(SerializerStates.IDLE);
              // m_robotContainer.spindexer.setState(SerializerStates.IDLE);
        }
        break;
    
    default:
      // Do nothing :)
      break;
    }


    gameData = DriverStation.getGameSpecificMessage();
    if(gameData.length() > 0)
    {
      switch (gameData.charAt(0))
      {
        case 'B' :
          //Blue case code
          break;
        case 'R' :
          //Red case code
          break;
        default :
          //This is corrupt data
          break;
      }
    } else {
      //Code for no data received yet
    }

    //logStates();

    // if(!hasBeenEnabled){
    //   SmartDashboard.putString("Dynamic Reading", m_robotContainer.dynamicAutoInput);
    // }
  }

  @Override
  public void disabledInit() {
    m_robotContainer.driverController.setRumble(RumbleType.kBothRumble, 0.0);
    m_robotContainer.setRobotState(RobotStates.IDLE);
   }
  

  @Override
  public void disabledPeriodic() {

    // Read dynamic auto selection from SmartDashboard
    m_robotContainer.dynamicAutoInput = SmartDashboard.getString("DynamicAutoInput", "");

    //System.out.println("DynamicAuto Input :" +m_robotContainer.dynamicAutoInput);

    Optional<Alliance> allianceColor = DriverStation.getAlliance();
  
   if (!hasBeenEnabled) {
    
    allianceColor.ifPresent(alliance -> {
      if (alliance == Alliance.Red) {
        m_robotContainer.drive.setHeading(180.0);
        //System.out.println("Red Alliance: Setting heading to 0 degrees.");
      } else if (alliance == Alliance.Blue) {
        m_robotContainer.drive.setHeading(0.0);
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
    m_robotContainer.setRobotState(RobotStates.IDLE);
  }

  @Override
  public void teleopPeriodic() {

    if (m_robotContainer.drive.isLocalized ){//(isHubActive()){
      m_robotContainer.driverController.setRumble(RumbleType.kBothRumble, 1.0);
    }
    else
    {
      m_robotContainer.driverController.setRumble(RumbleType.kBothRumble, 0.0);
    }

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

  public boolean isHubActive() {
  Optional<Alliance> alliance = DriverStation.getAlliance();
  // If we have no alliance, we cannot be enabled, therefore no hub.
  if (alliance.isEmpty()) {
    return false;
  }
  // Hub is always enabled in autonomous.
  if (DriverStation.isAutonomousEnabled()) {
    return true;
  }
  // At this point, if we're not teleop enabled, there is no hub.
  if (!DriverStation.isTeleopEnabled()) {
    return false;
  }

  // We're teleop enabled, compute.
  double matchTime = DriverStation.getMatchTime();
  String gameData = DriverStation.getGameSpecificMessage();
  // If we have no game data, we cannot compute, assume hub is active, as its likely early in teleop.
  if (gameData.isEmpty()) {
    return true;
  }
  boolean redInactiveFirst = false;
  switch (gameData.charAt(0)) {
    case 'R' -> redInactiveFirst = true;
    case 'B' -> redInactiveFirst = false;
    default -> {
      // If we have invalid game data, assume hub is active.
      return true;
    }
  }

  // Shift was is active for blue if red won auto, or red if blue won auto.
  boolean shift1Active = switch (alliance.get()) {
    case Red -> !redInactiveFirst;
    case Blue -> redInactiveFirst;
  };

  if (matchTime > 130) {
    // Transition shift, hub is active.
    return true;
  } else if (matchTime > 105) {
    // Shift 1
    return shift1Active;
  } else if (matchTime > 80) {
    // Shift 2
    return !shift1Active;
  } else if (matchTime > 55) {
    // Shift 3
    return shift1Active;
  } else if (matchTime > 30) {
    // Shift 4
    return !shift1Active;
  } else {
    // End game, hub always active.
    return true;
  }
}

}


