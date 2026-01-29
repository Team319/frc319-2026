// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.util.Alert;
import frc.robot.util.Alert.*;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/** Add your docs here. */
public class Constants {

  // Enable this for Tunable Values
  public static final boolean tuningMode = false;
  public static final double loopPeriodSecs = 0.02;

  private static RobotType robotType = RobotType.DEVBOT;

  private static FieldType fieldType = FieldType.ANDYMARK;

  private static DemoMode demoMode = DemoMode.OFF;
  

  public static RobotType getRobot() {
    if ( RobotBase.isReal() && robotType == RobotType.SIMBOT) {
      new Alert("Invalid robot selected, using competition robot as default.", AlertType.ERROR)
          .set(true);
      robotType = RobotType.COMPBOT;
    }else if(RobotBase.isSimulation()){
      robotType = RobotType.SIMBOT;
    }
    // else the default was COMPBOT
    return robotType;
  }

  public static Mode getMode() {
    return switch (robotType) {
      case DEVBOT, COMPBOT -> RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
      case SIMBOT -> Mode.SIM;
    };
  }

  public static FieldType getFieldType() {
    return fieldType;
  }

  public static DemoMode getDemoMode() {
    return demoMode;
  }

  public enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public enum RobotType {
    SIMBOT,
    DEVBOT,
    COMPBOT
  }

  public enum FieldType {
    ANDYMARK,
    WELDED
  }

  public enum DemoMode {
    OFF,
    ON
  } 

// =========================================================================
//                          DRIVETRAIN CONSTANTS
// =========================================================================

  public static class DriveConstants{
    public static final double MAX_LINEAR_SPEED = Units.feetToMeters(17.1); 
    public static final double TRACK_WIDTH_X = Units.inchesToMeters(22); 
    public static final double TRACK_WIDTH_Y = Units.inchesToMeters(22); 
    public static final double DRIVE_BASE_RADIUS =
        Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
    public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;
    public static final int currentLimit = 60;
    public static final double wheelRadiusMeters = Units.inchesToMeters(3.875); 
    public static final double robotMassKg = 52.16312; 
    public static final double robotMOI = 6.883; 
    public static final double wheelCOF = 1.2; 
    public static final double DRIVE_GEAR_RATIO = 6.122; //(50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0); // L3
    public static final double TURN_GEAR_RATIO = 150.0 / 7.0;
    public static final boolean isTurnMotorInverted = true;

    public static final PathConstraints pathingConstraints = new PathConstraints(
        2.0,2.0,
        //2.0, 2.0, // at battlecry
        Units.degreesToRadians(540), Units.degreesToRadians(720));

    public static final PathConstraints testingPathingConstraints = new PathConstraints(
        0.2,0.2,
        //3.0, 4.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720));

        public static final PathConstraints autoPathingConstraints = new PathConstraints(
          2.0,2.0,
          //3.0, 4.0,
          Units.degreesToRadians(540), Units.degreesToRadians(720));

    private static final double DEMO_MODE_SPEED_FACTOR = 0.5;
    public static final double DEMO_MODE_MAX_LINEAR_SPEED_METERS_PER_SEC = MAX_LINEAR_SPEED * DEMO_MODE_SPEED_FACTOR;
    public static final double DEMO_MODE_MAX_ANGULAR_SPEED_RAD_PER_SEC = MAX_ANGULAR_SPEED * DEMO_MODE_SPEED_FACTOR;

  } // End of DriveConstants

  public static enum HeadingTargets{
    NO_TARGET,
    HUB
  } // End of HeadingTargets

  public static class TargetLocations{
    public static final double FIELD_LENGTH = Units.feetToMeters(54);
    public static final double FIELD_WIDTH = Units.feetToMeters(27);
    
    // =============== COMMON TARGET POSES ===============
    public static Pose2d ORIGIN = new Pose2d();
    public static Pose2d CENTER_OF_FIELD = new Pose2d();

    // =============== BLUE SIDE TARGET POSES ===============
    public static Pose2d BLUE_HUB = new Pose2d();

    // =============== RED SIDE TARGET POSES ===============
    public static Pose2d RED_HUB = new Pose2d();
  
  } // End of TargetLocations

  public static class LimelightConstants{
    public static enum Device{
      DRIVETRAIN,
      TURRET,
      COLLECTOR
    }
  
  } // End of LimelightConstants

// =========================================================================
//                          LAUNCHER CONSTANTS
// =========================================================================

  public static class LauncherConstants{

    public static final double TURRET_MIN_ANGLE_RADIANS = Units.degreesToRadians(-180);
    public static final double TURRET_MAX_ANGLE_RADIANS = Units.degreesToRadians(180);

    public static final double HOOD_MIN_ANGLE_RADIANS = Units.degreesToRadians(0);
    public static final double HOOD_MAX_ANGLE_RADIANS = Units.degreesToRadians(60);

    public static final double FLYWHEEL_MAX_VELOCITY_RPM = 5000.0;
    public static final double KICKER_MAX_VELOCITY_RPM = 5000.0;
  
  } // End of LauncherConstants

} // End of Constants
