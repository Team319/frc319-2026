// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc319.robot;

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
import frc319.redhawk_lib.io.SimTalonFXIO;
import frc319.redhawk_lib.io.TalonFXIO;
import frc319.redhawk_lib.subsystem.KinematicsManager;
import frc319.robot.Constants.DriveConstants;
import frc319.robot.commands.DriveCommands;
import frc319.robot.commands.autos.DynamicAutoRoutine;
import frc319.robot.subsystems.drive.Drive;
import frc319.robot.subsystems.drive.GyroIO;
import frc319.robot.subsystems.drive.GyroIOPigeon2;
import frc319.robot.subsystems.drive.ModuleIOSim;
import frc319.robot.subsystems.drive.ModuleIOTalonFX;
import frc319.robot.subsystems.intake.IntakeConstants;
import frc319.robot.subsystems.launcher.Turret;
import frc319.robot.subsystems.launcher.Flywheels;
import frc319.robot.subsystems.launcher.Hood;
import frc319.robot.subsystems.serializer.BallTunnel;
import frc319.robot.subsystems.serializer.SerializerConstants;
import frc319.robot.subsystems.serializer.Spindexer;
import frc319.robot.subsystems.launcher.LauncherConstants;
import frc319.robot.subsystems.launcher.LaunchingSolutionManager;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.CommandScheduler;



public class RobotContainer {

  private final KinematicsManager kinematicsManager = new KinematicsManager();
  private final LaunchingSolutionManager launchingSolutionManager = new LaunchingSolutionManager();

  // Subsystems
  public final Drive drive;  
  public final Turret turret;
  public final Flywheels flywheels;
  public final Hood hood;
  public final BallTunnel ballTunnel;
  public final Spindexer spindexer;
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
        case DEVBOT:

          drive =
            new Drive(
                 new GyroIOPigeon2() {},
                 new ModuleIOTalonFX(0),
                 new ModuleIOTalonFX(1),
                 new ModuleIOTalonFX(2),
                 new ModuleIOTalonFX(3)); 

          turret =
            new Turret(
                LauncherConstants.Turret.config, new TalonFXIO(LauncherConstants.Turret.config));
          ballTunnel =
            new BallTunnel(SerializerConstants.BallTunnel.config, new SimTalonFXIO(SerializerConstants.BallTunnel.config));
          
          spindexer = 
            new Spindexer(SerializerConstants.Spindexer.config, new SimTalonFXIO(SerializerConstants.Spindexer.config));         
            
          flywheels =
            new Flywheels(
                LauncherConstants.Flywheels.leftConfig,
                LauncherConstants.Flywheels.rightConfig,
                new SimTalonFXIO(LauncherConstants.Flywheels.leftConfig),
                new SimTalonFXIO(LauncherConstants.Flywheels.rightConfig));

          hood =
            new Hood(
              LauncherConstants.Hood.config, new SimTalonFXIO(LauncherConstants.Hood.config));


          break;

        case LAUNCHER_PROTOTYPE:
              // Simulated drivetrain
              drive =
              new Drive(
                  new GyroIO() {},
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim() );
            // Simulated Turret
            turret =
              new Turret(
                  LauncherConstants.Turret.config, new SimTalonFXIO(LauncherConstants.Turret.config));

        flywheels =
            new Flywheels(
                LauncherConstants.Flywheels.leftConfig,
                LauncherConstants.Flywheels.rightConfig,
                new TalonFXIO(LauncherConstants.Flywheels.leftConfig),
                new TalonFXIO(LauncherConstants.Flywheels.rightConfig));

            ballTunnel =
              new BallTunnel(SerializerConstants.BallTunnel.config, new TalonFXIO(SerializerConstants.BallTunnel.config));
            
            spindexer = 
              new Spindexer(SerializerConstants.Spindexer.config, new TalonFXIO(SerializerConstants.Spindexer.config));  

            hood =
              new Hood(
                LauncherConstants.Hood.config, new TalonFXIO(LauncherConstants.Hood.config));



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

            turret =
              new Turret(
                  LauncherConstants.Turret.config, new SimTalonFXIO(LauncherConstants.Turret.config));

        flywheels =
            new Flywheels(
                LauncherConstants.Flywheels.leftConfig,
                LauncherConstants.Flywheels.rightConfig,
                new SimTalonFXIO(LauncherConstants.Flywheels.leftConfig),
                new SimTalonFXIO(LauncherConstants.Flywheels.rightConfig));
            hood =
              new Hood(
                LauncherConstants.Hood.config, new SimTalonFXIO(LauncherConstants.Hood.config));
             
          ballTunnel =
              new BallTunnel(SerializerConstants.BallTunnel.config, new SimTalonFXIO(SerializerConstants.BallTunnel.config));
 
            
          spindexer = 
              new Spindexer(SerializerConstants.Spindexer.config, new SimTalonFXIO(SerializerConstants.Spindexer.config)); 
            
            break;
      } // End of robot-specific subsystem instantiation switch statement

      // ==========================================
      // Set up for Elastic dashboard for testing
      // ==========================================

      // The slider will be available in Elastic's dashboard
      DoubleSupplier flywheelTestRPM = () -> SmartDashboard.getNumber("Flywheel Test RPM", 0.0);
      flywheels.setTestFlywheelRPMSupplier(flywheelTestRPM);
      
      DoubleSupplier ballTunnelTestDutyCycle = () -> SmartDashboard.getNumber("BallTunnel Test DutyCycle", 0.0);
      ballTunnel.setTestBallTunnelDutyCycle(ballTunnelTestDutyCycle);
      DoubleSupplier SpindexerTestDutyCycle = () -> SmartDashboard.getNumber("SpindexerTestDutyCycle", 0.0);
      spindexer.setTestSpindexerDutyCycle(SpindexerTestDutyCycle);
      // Initialize the slider with a default value and bounds (optional but recommended)
      SmartDashboard.putNumber("Flywheel Test RPM", 0.0);  // Default value
      SmartDashboard.putNumber("BallTunnel Test DutyCycle", 0.0);  // Default value
       SmartDashboard.putNumber("SpindexerTestDutyCycle", 0.0);
      // ==========================================
      // End of Elastic dashboard setup
      // ==========================================


      //Set up Named Commands in Pathplanner

      // NamedCommands.registerCommand(
      //   "Collect",
      //   new CollectCoral(superstructure));

      // Set up auto routines
      autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

      autoChooser.addOption("DynamicAutoRoutine", null);
      
      // Add Commands to the dashboard chooser
      //autoChooser.addOption(
      //    "Name on Dashboard", Commands);
  
      configureBindings();

    // configure the kinematics calculations
    configureKinematics();
  }

  /** Use this robot to configure the transforms between subsystems. */
  private void configureKinematics() {
    
    kinematicsManager.registerUnpublished(drive, 0, -1);

    kinematicsManager.register(
        turret, LauncherConstants.Turret.MODEL_INDEX, LauncherConstants.Turret.PARENT_INDEX);

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

          driverController.rightBumper()
            .onTrue(new InstantCommand(()->turret.setState(LauncherConstants.Turret.TurretState.TRACK_HUB_ON_MOVE)))
            .onFalse(new InstantCommand(()->turret.setState(LauncherConstants.Turret.TurretState.IDLE)));

        driverController.a()
          .onTrue( hood.setAngle(() -> Degrees.of(0)) );
          //.onFalse( hood.setAngle(() -> Degrees.of(0)) );

        driverController.b()
          .onTrue( hood.setAngle(() -> Degrees.of(15)) );
          //.onFalse( hood.setAngle(() -> Degrees.of(0)) );

        driverController.y()
          .onTrue( hood.setAngle(() -> Degrees.of(25)) );
          //.onFalse( hood.setAngle(() -> Degrees.of(0)) );

        driverController.povRight()
          .onTrue( flywheels.setVelocity(() -> RPM.of(1000)) );

        driverController.povUp()
          .onTrue( flywheels.setVelocity(() -> RPM.of(2000)) );

        driverController.povLeft()
          .onTrue( flywheels.setVelocity(() -> RPM.of(3000)) );

        driverController.povDown()
          .onTrue( flywheels.setVelocity(() -> RPM.of(0)) );

        driverController.back()
          .onTrue( new InstantCommand(()->flywheels.setState(LauncherConstants.Flywheels.FlywheelsState.TESTING_ENABLED)))
          .onFalse( new InstantCommand(()->flywheels.setState(LauncherConstants.Flywheels.FlywheelsState.IDLE)));
          

        // driverController.x().onTrue( turret.setAngle(() -> Degrees.of(90)) );  
        // driverController.y().onTrue( turret.setAngle(() -> Degrees.of(180)) );
        // driverController.b().onTrue( turret.setAngle(() -> Degrees.of(360)) );

        //driverController.rightBumper().whileTrue( new InstantCommand(() -> turret.setAngle(() -> turret.getLauncOnTheFlyAngle())) );
        
        if (true){ // Turned off for safety at demos

          // TODO 1/22/26 - test these, as the schedule method changed
          
          // TODO - add pathfinding under the closest trench

          driverController.leftBumper().whileTrue( new InstantCommand(()-> CommandScheduler.getInstance().schedule( DriveCommands.pathfindUnderNearestTrench(drive)) ));
          
        }

        //  ===========================================================================
        //  ============================= Operator Controls ===========================
        //  ===========================================================================

    }
  public Command getAutonomousCommand() {
    
    if(autoChooser.get() == null){

      // TODO - Will probably hard code the Dynamic Autos this year... 
      
      // TODO - Potentially implement behaviorTreeAuto routine instead of DynamicAutoRoutine

      // The command needs to be created at runtime so that the formatted 
      //  instruction string is populated from the dashboard
      return new DynamicAutoRoutine(drive); 
    }
    else{
      return autoChooser.get();
    }
    
    


  }
}