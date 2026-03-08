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
import frc319.lib.io.SimTalonFXIO;
import frc319.lib.io.TalonFXIO;
import frc319.lib.subsystem.KinematicsManager;
import frc319.robot.Constants.DemoMode;
import frc319.robot.Constants.DriveConstants;
import frc319.robot.commands.DriveCommands;
import frc319.robot.commands.autos.DynamicAutoRoutine;
import frc319.robot.subsystems.drive.Drive;
import frc319.robot.subsystems.drive.GyroIO;
import frc319.robot.subsystems.drive.GyroIOPigeon2;
import frc319.robot.subsystems.drive.ModuleIOSim;
import frc319.robot.subsystems.drive.ModuleIOTalonFX;
import frc319.robot.subsystems.intake.IntakeConstants;
import frc319.robot.subsystems.intake.IntakeExtension;
import frc319.robot.subsystems.intake.IntakeRollers;
import frc319.robot.subsystems.intake.IntakeConstants.IntakeStates;
import frc319.robot.subsystems.launcher.Turret;
import frc319.robot.subsystems.launcher.LauncherConstants.LauncherStates;
import frc319.robot.subsystems.launcher.LauncherConstants.Flywheels.FlywheelsState;
import frc319.robot.subsystems.launcher.Flywheels;
import frc319.robot.subsystems.launcher.Hood;
import frc319.robot.subsystems.serializer.BallTunnel;
import frc319.robot.subsystems.serializer.SerializerConstants;
import frc319.robot.subsystems.serializer.Spindexer;
import frc319.robot.subsystems.serializer.SerializerConstants.SerializerStates;
import frc319.robot.subsystems.launcher.LauncherConstants;
import frc319.robot.subsystems.launcher.LaunchingSolutionManager;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.io.Serial;
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
  public final IntakeExtension intakeExtension;
  public final IntakeRollers intakeRollers;
  // Controller
  public final CommandXboxController driverController = new CommandXboxController(0);
  public final CommandXboxController operatorController = new CommandXboxController(1);

  //Dynamic Auto Routine Input String
  public String dynamicAutoInput = "";

  public RobotStates currentRobotState = RobotStates.IDLE;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser; // AdvantageKit Dependency
    
    public RobotContainer() {
      switch(Constants.getRobot()){
  
        case COMPBOT:

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
            new BallTunnel(SerializerConstants.BallTunnel.config, new TalonFXIO(SerializerConstants.BallTunnel.config));
          
          spindexer = 
            new Spindexer(SerializerConstants.Spindexer.config, new TalonFXIO(SerializerConstants.Spindexer.config));         
            
          flywheels =
            new Flywheels(
                LauncherConstants.Flywheels.leftConfig,
                LauncherConstants.Flywheels.rightConfig,
                new TalonFXIO(LauncherConstants.Flywheels.leftConfig),
                new TalonFXIO(LauncherConstants.Flywheels.rightConfig));

          hood =
            new Hood(
              LauncherConstants.Hood.config, new TalonFXIO(LauncherConstants.Hood.config));

          intakeExtension =
            new IntakeExtension(
              IntakeConstants.Extension.config, new TalonFXIO(IntakeConstants.Extension.config));

          intakeRollers =
            new IntakeRollers(
              IntakeConstants.Rollers.leadConfig, IntakeConstants.Rollers.followConfig,
              new TalonFXIO(IntakeConstants.Rollers.leadConfig), new TalonFXIO(IntakeConstants.Rollers.followConfig));

          break;

          case DEVBOT:
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

          intakeExtension =
            new IntakeExtension(
              IntakeConstants.Extension.config, new SimTalonFXIO(IntakeConstants.Extension.config));

          intakeRollers =
            new IntakeRollers(
              IntakeConstants.Rollers.leadConfig, IntakeConstants.Rollers.followConfig,
              new SimTalonFXIO(IntakeConstants.Rollers.leadConfig), new SimTalonFXIO(IntakeConstants.Rollers.followConfig));
            
            break;
      } // End of robot-specific subsystem instantiation switch statement

      // ==========================================
      // Set up for Elastic dashboard for testing
      // ==========================================

      // The slider will be available in Elastic's dashboard
      // DoubleSupplier flywheelTestRPM = () -> SmartDashboard.getNumber("Flywheel Test RPM", 0.0);
      // flywheels.setTestFlywheelRPMSupplier(flywheelTestRPM);
      
      // DoubleSupplier ballTunnelTestDutyCycle = () -> SmartDashboard.getNumber("BallTunnel Test DutyCycle", 0.0);
      // ballTunnel.setTestBallTunnelDutyCycle(ballTunnelTestDutyCycle);
      // DoubleSupplier SpindexerTestDutyCycle = () -> SmartDashboard.getNumber("SpindexerTestDutyCycle", 0.0);
      // spindexer.setTestSpindexerDutyCycle(SpindexerTestDutyCycle);
      // // Initialize the slider with a default value and bounds (optional but recommended)
      // SmartDashboard.putNumber("Flywheel Test RPM", 0.0);  // Default value
      // SmartDashboard.putNumber("BallTunnel Test DutyCycle", 0.0);  // Default value
      //  SmartDashboard.putNumber("SpindexerTestDutyCycle", 0.0);

      SmartDashboard.putNumber("Tuning_Mode/Flywheel_Tuning_RPM", 0.0);
      SmartDashboard.putNumber("Tuning_Mode/Hood_Tuning_Position_Degrees", 0.0);
      SmartDashboard.putNumber("Tuning_Mode/Turret_Tuning_Position_Degrees", 0.0);
      SmartDashboard.putNumber("Tuning_Mode/Intake_Tuning_Position_Inches", 0.0);

      SmartDashboard.putNumber("Tuning_Mode/Spindexer_Tuning_RPM", 0.0);
      SmartDashboard.putNumber("Tuning_Mode/BallTunnel_Tuning_RPM", 0.0);

      // DoubleSupplier flywheelTuningRPM = () -> SmartDashboard.getNumber("/Tuning/Flywheel_Tuning_RPM", 0.0);
      // DoubleSupplier hoodTuningPosition = () -> SmartDashboard.getNumber("/Tuning/Hood_Tuning_Position_Degrees", 0.0);
      // DoubleSupplier turretTuningPosition = () -> SmartDashboard.getNumber("/Tuning/Turret_Tuning_Position_Degrees", 0.0);
      // DoubleSupplier intakeTuningPosition = () -> SmartDashboard.getNumber("/Tuning/Intake_Tuning_Position_Inches", 0.0);



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
        intakeExtension, IntakeConstants.Extension.MODEL_INDEX, IntakeConstants.Extension.PARENT_INDEX);

    kinematicsManager.register(
        spindexer, SerializerConstants.Spindexer.MODEL_INDEX, SerializerConstants.Spindexer.PARENT_INDEX);

    kinematicsManager.register(
        turret, LauncherConstants.Turret.MODEL_INDEX, LauncherConstants.Turret.PARENT_INDEX);

    kinematicsManager.register(
        hood, LauncherConstants.Hood.MODEL_INDEX, LauncherConstants.Hood.PARENT_INDEX);

    kinematicsManager.register(
        flywheels, LauncherConstants.Flywheels.MODEL_INDEX, LauncherConstants.Flywheels.PARENT_INDEX);

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


        driverController.rightTrigger().whileTrue( new InstantCommand(()->this.setRobotState(RobotStates.SHOOTING)) )
         .onFalse( new InstantCommand(()->this.setRobotState(RobotStates.STOWED)) );

        // driverController.rightBumper()
        //   .onTrue(new InstantCommand(()->turret.setState(LauncherConstants.LauncherStates.TRACK_HUB_ON_MOVE)))
        //   .onFalse(new InstantCommand(()->turret.setState(LauncherConstants.LauncherStates.IDLE)));

        // driverController.rightBumper()
        //   .onTrue(turret.setAngle(()-> Degrees.of(720)))
        //   .onFalse(turret.setAngle(()-> Degrees.of(0)));

        // driverController.povDown()
        //   .onTrue( new InstantCommand(()->this.setRobotState(RobotStates.IDLE)));

        driverController.povRight()
          .onTrue( new InstantCommand(()->this.setRobotState(RobotStates.TUNING_STOP)));

        // driverController.povUp()
        //   .onTrue( new InstantCommand(()->this.setRobotState(RobotStates.SNOWBLOW)));

        driverController.povLeft()
          .onTrue( new InstantCommand(()->this.setRobotState(RobotStates.TUNING_SHOOT)));

        //  driverController.back()
        //    .onTrue( new InstantCommand(()->this.setRobotState(RobotStates.TUNING)));
        //   .onFalse( new InstantCommand(()->flywheels.setState(LauncherConstants.Flywheels.FlywheelsState.IDLE)));

        
        // if (Constants.getDemoMode() == DemoMode.OFF){ // Auto Pathing Turned off for safety at demos
        //   driverController.leftBumper().onTrue( new InstantCommand(()-> this.setRobotState(RobotStates.TRENCH)));
        //   driverController.leftBumper().whileTrue( new InstantCommand(()-> CommandScheduler.getInstance().schedule( DriveCommands.pathfindUnderNearestTrench(drive))));
          
        // }

        //  ===========================================================================
        //  ============================= Operator Controls ===========================
        //  ===========================================================================

    }
  public Command getAutonomousCommand() {
    
    if(autoChooser.get() == null){

      // TODO - Will probably hard code the Dynamic Autos options this year... 
      
      // The command needs to be created at runtime so that the formatted 
      //  instruction string is populated from the dashboard
      return new DynamicAutoRoutine(drive); 
    }
    else{
      return autoChooser.get();
    }
  } // End of getAutonomousCommand()

  //  ===========================================================================
  //  ========================        Robot States      =========================
  //  ===========================================================================

  // probably smart to make 

  public enum RobotStates{
    IDLE,
    STOWED,
    COLLECTING,
    SHOOTING,
    SNOWBLOW, // Means collecting while shooting at the desired target. probably want to have some logic when to gracefully switch while shooting
    TRENCH,
    //TUNING
    TUNING_SHOOT,
    TUNING_STOP
  }

  public RobotStates getCurrentRobotState(){
    return currentRobotState;
  }

  public void setRobotState(RobotStates state){

    currentRobotState = state;

    switch(state){

      case STOWED:
        intakeExtension.setState(IntakeStates.RETRACTED);
        intakeRollers.setState(IntakeStates.IDLE);
        ballTunnel.setState(SerializerStates.IDLE);
        flywheels.setState(FlywheelsState.PRESPIN);
        spindexer.setState(SerializerStates.IDLE);
        turret.setState(LauncherStates.STOWED);
        hood.setState(LauncherStates.STOWED);
        break;

      case COLLECTING:
        intakeExtension.setState(IntakeStates.EXTENDED);
        intakeRollers.setState(IntakeStates.COLLECT); 
        ballTunnel.setState(SerializerStates.IDLE);
        flywheels.setState(FlywheelsState.PRESPIN);
        spindexer.setState(SerializerStates.IDLE); // Maybe jostle hopper while collecting?
        turret.setState(LauncherStates.STOWED);
        hood.setState(LauncherStates.STOWED);
        break;

      case SHOOTING:
        flywheels.setState(FlywheelsState.SHOOT);
        // ballTunnel.setState(SerializerStates.SHOOT);
        // spindexer.setState(SerializerStates.SHOOT);
        turret.setState(LauncherStates.TRACKING_TARGET);
        hood.setState(LauncherStates.TRACKING_TARGET);
        break;

      case SNOWBLOW:
        intakeExtension.setState(IntakeStates.EXTENDED);
        intakeRollers.setState(IntakeStates.COLLECT);
        // ballTunnel.setState(SerializerStates.SHOOT);
        flywheels.setState(FlywheelsState.SHOOT);        
        // spindexer.setState(SerializerStates.SHOOT);
        turret.setState(LauncherStates.TRACKING_TARGET);
        hood.setState(LauncherStates.TRACKING_TARGET);
        break;

      case TRENCH:
        // Just ensure the turret is STOWED... 
        hood.setState(LauncherStates.STOWED);

        // Should probably stop shooting too...
        ballTunnel.setState(SerializerStates.IDLE);
        flywheels.setState(FlywheelsState.PRESPIN);
        spindexer.setState(SerializerStates.IDLE);   

        break;

      case TUNING_SHOOT:
        // Implement tuning state behavior here
          intakeExtension.setState(IntakeStates.TUNING);
          //intakeRollers.setState(IntakeStates.TUNING);
          ballTunnel.setState(SerializerStates.SHOOT);
          flywheels.setState(FlywheelsState.TUNING); 
          spindexer.setState(SerializerStates.SHOOT);
          turret.setState(LauncherStates.TUNING);
          hood.setState(LauncherStates.TUNING);
        break;

      case TUNING_STOP:
        // Implement tuning state behavior here
          intakeExtension.setState(IntakeStates.TUNING);
          //intakeRollers.setState(IntakeStates.TUNING);
          ballTunnel.setState(SerializerStates.IDLE);
          flywheels.setState(FlywheelsState.TUNING); 
          spindexer.setState(SerializerStates.IDLE);
          turret.setState(LauncherStates.TUNING);
          hood.setState(LauncherStates.TUNING);
        break;

      case IDLE:
      default:  
        intakeExtension.setState(IntakeStates.IDLE);
        intakeRollers.setState(IntakeStates.IDLE);
        ballTunnel.setState(SerializerStates.IDLE);
        flywheels.setState(FlywheelsState.IDLE); 
        spindexer.setState(SerializerStates.IDLE);
        turret.setState(LauncherStates.IDLE);
        hood.setState(LauncherStates.IDLE);
        break;
    }

  }

}