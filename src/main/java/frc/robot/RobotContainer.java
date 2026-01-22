// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import edu.wpi.first.cameraserver.CameraServer;
//import edu.wpi.first.cscore.UsbCamera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DriveCommands;

import frc.robot.commands.autos.DynamicAutoRoutine;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;

import frc.robot.subsystems.drive.GyroIOPigeon2;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.CommandScheduler;



public class RobotContainer {

 // private UsbCamera camera;
  // Subsystems
  public final Drive drive;  

  // Controller
  public final CommandXboxController driverController = new CommandXboxController(0);
  public final CommandXboxController operatorController = new CommandXboxController(1);

  //Dynamic Auto Routine Input String
  public String dynamicAutoInput = "";

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser; // AdvantageKit Dependency
    
    public RobotContainer() {
      switch(Constants.getRobot()){
  
        case COMPBOT:

          // camera = CameraServer.startAutomaticCapture();
          // camera.setResolution(640, 480);
          // camera.setFPS(30);

          drive =
            new Drive(
                 new GyroIOPigeon2() {},
                 new ModuleIOTalonFX(0),
                 new ModuleIOTalonFX(1),
                 new ModuleIOTalonFX(2),
                 new ModuleIOTalonFX(3)); 

          break;

        case DEVBOT:

          drive =
          new Drive(
               new GyroIO() {},
               new ModuleIOTalonFX(0),
               new ModuleIOTalonFX(1),
               new ModuleIOTalonFX(2),
               new ModuleIOTalonFX(3)
               ); 
          break;

          case SIMBOT:
          default:
            // Sim robot, instantiate physics sim IO implementations
            drive =
              new Drive(
                  new GyroIO() {},
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim() );
                  
            break;
      }
      //Set up Named Commands in Pathplanner

      // NamedCommands.registerCommand(
      //   "Collect",
      //   new CollectCoral(superstructure));

      // NamedCommands.registerCommand(
      //   "ScoreL4",
      //   new SafelyMoveToScoringPosition(superstructure, 4));

      // NamedCommands.registerCommand(
      //   "ScoreCoral",
      //   new AutoScoreCoral(superstructure));

      // NamedCommands.registerCommand(
      //   "GoHome",
      //   new AutoGoHome(superstructure));

      // Set up auto routines
      autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

      autoChooser.addOption("DynamicAutoRoutine", null);
      
      // Add Commands to the dashboard chooser
      //autoChooser.addOption(
      //    "Name on Dashboard", Commands);
  
      configureBindings();
    
    }
  
    private void configureBindings() {
      switch(Constants.getRobot()){
        case SIMBOT:
        case DEVBOT:
        case COMPBOT:
        default:

        //  ===========================================================================
        //  ============================= Driver Controls =============================
        //  ===========================================================================

        /*  ============================= Drive ============================= */
  
        drive.setDefaultCommand(
          DriveCommands.joystickDrive(
              drive,
              () -> -driverController.getLeftY(), // Note : This is X supplier because the field's X axis is down field long
              () -> -driverController.getLeftX(), // Note this is Y supplier because the field's Y axis is across the field 
              () -> -driverController.getRightY(), 
              () -> -driverController.getRightX(),
              () -> driverController.getLeftTriggerAxis()));
        
        break;
      }
  
        driverController.start().whileTrue(Commands.runOnce(
            ()-> { 
              drive.resetHeading();
            }
            )
          );


        if (true){ // Turned off for safety at demos

          driverController.rightBumper().whileTrue(   new InstantCommand(()-> CommandScheduler.getInstance().schedule( drive.pathfindToClosestRightReef() )) );  // TODO 1/22/26 - test these, as the schedule method changed

          driverController.leftBumper().whileTrue( new InstantCommand(()-> CommandScheduler.getInstance().schedule( drive.pathfindToClosestLeftReef() )) );
  
         // driverController.back().whileTrue( new InstantCommand(()-> drive.pathfindToProcessor().schedule()  ) );
        
        }

        //  ===========================================================================
        //  ============================= Operator Controls ===========================
        //  ===========================================================================

    }
  public Command getAutonomousCommand() {
    
    if(autoChooser.get() == null){
      return new DynamicAutoRoutine(drive); // The command needs to be created at runtime so that the instruction string is populated from the dashboard
    }
    else{
      return autoChooser.get();
    }
    
    


  }
}