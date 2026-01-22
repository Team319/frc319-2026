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

  public static class DriveConstants{
    public static final double MAX_LINEAR_SPEED = Units.feetToMeters(17.1); 
    public static final double TRACK_WIDTH_X = Units.inchesToMeters(22); 
    public static final double TRACK_WIDTH_Y = Units.inchesToMeters(22); 
    public static final double DRIVE_BASE_RADIUS =
        Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
    public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;
    public static final int currentLimit = 40; // TODO consider setting this to 60
    public static final double wheelRadiusMeters = Units.inchesToMeters(3.875); // Black nitrile : 3 7/8 with full tread = 3.875 inches
    public static final double robotMassKg = 52.16312; // TODO
    public static final double robotMOI = 6.883; // TODO
    public static final double wheelCOF = 1.2; // TODO
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

  }
  public static enum HeadingTargets{
    NO_TARGET,
    REEF_CENTER,
    CORAL_STATION_LEFT,
    CORAL_STATION_RIGHT,
    PROCESSOR
  }

  public static class TargetLocations{
    public static final double FIELD_LENGTH = Units.feetToMeters(54);
    public static final double FIELD_WIDTH = Units.feetToMeters(27);
    
    // =============== COMMON TARGET POSES ===============
    public static Pose2d ORIGIN = new Pose2d();
    public static Pose2d CENTER_OF_FIELD = new Pose2d();

    // =============== BLUE SIDE TARGET POSES ===============
    public static Pose2d BLUE_REEF_CENTER = new Pose2d();
    public static Pose2d BLUE_REEF_A = new Pose2d();
    public static Pose2d BLUE_REEF_B = new Pose2d();
    public static Pose2d BLUE_REEF_C = new Pose2d();
    public static Pose2d BLUE_REEF_D = new Pose2d();
    public static Pose2d BLUE_REEF_E = new Pose2d();
    public static Pose2d BLUE_REEF_F = new Pose2d();
    public static Pose2d BLUE_REEF_G = new Pose2d();
    public static Pose2d BLUE_REEF_H = new Pose2d();
    public static Pose2d BLUE_REEF_I = new Pose2d();
    public static Pose2d BLUE_REEF_J = new Pose2d();
    public static Pose2d BLUE_REEF_K = new Pose2d();
    public static Pose2d BLUE_REEF_L = new Pose2d();

    public static Pose2d BLUE_CORAL_STATION_LEFT = new Pose2d();
    public static Pose2d BLUE_CORAL_STATION_RIGHT = new Pose2d();
    public static Pose2d BLUE_SIDE_PROCESSOR = new Pose2d(6.0,0.5,new Rotation2d(Units.degreesToRadians(0))); // Technically this is red... but it's on the blue side!

    public static Pose2d BLUE_START_LEFT = new Pose2d(7.5,6.5,new Rotation2d(Units.degreesToRadians(180)));
    public static Pose2d BLUE_START_CENTER = new Pose2d(7.5,4.0,new Rotation2d(Units.degreesToRadians(180)));
    public static Pose2d BLUE_START_RIGHT = new Pose2d(7.5,1.5,new Rotation2d(Units.degreesToRadians(180)));

    // =============== RED SIDE TARGET POSES ===============
    public static Pose2d RED_REEF_CENTER = new Pose2d();
    public static Pose2d RED_REEF_A = new Pose2d();
    public static Pose2d RED_REEF_B = new Pose2d();
    public static Pose2d RED_REEF_C = new Pose2d();
    public static Pose2d RED_REEF_D = new Pose2d();
    public static Pose2d RED_REEF_E = new Pose2d();
    public static Pose2d RED_REEF_F = new Pose2d();
    public static Pose2d RED_REEF_G = new Pose2d();
    public static Pose2d RED_REEF_H = new Pose2d();
    public static Pose2d RED_REEF_I = new Pose2d();
    public static Pose2d RED_REEF_J = new Pose2d();
    public static Pose2d RED_REEF_K = new Pose2d();
    public static Pose2d RED_REEF_L = new Pose2d();

    public static Pose2d RED_CORAL_STATION_LEFT = new Pose2d(); 
    public static Pose2d RED_CORAL_STATION_RIGHT = new Pose2d();
    public static Pose2d RED_SIDE_PROCESSOR = new Pose2d(11.5,7.5,new Rotation2d(Units.degreesToRadians(180))); // Technically this is blue... but it's on the red side!

    public static Pose2d RED_START_LEFT = new Pose2d(10.0,1.5,new Rotation2d(Units.degreesToRadians(0)));
    public static Pose2d RED_START_CENTER = new Pose2d(10.0,4.0,new Rotation2d(Units.degreesToRadians(0)));
    public static Pose2d RED_START_RIGHT = new Pose2d(10.0,6.5,new Rotation2d(Units.degreesToRadians(0)));
    
  }

  public static class LimelightConstants{
    public static enum Device{
      REEF,
      CORAL_STATION
    }
  }

}
