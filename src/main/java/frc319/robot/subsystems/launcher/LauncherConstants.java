package frc319.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import frc319.lib.drivers.CANDeviceId;
import frc319.lib.dynamics.MoiUnits;
import frc319.lib.subsystem.TalonFXSubsystemConfig;
import frc319.lib.util.LoggedTunableBoolean;
import frc319.lib.util.LoggedTunableMeasure;
import frc319.robot.FieldConstants;

public final class LauncherConstants {

    public static enum LauncherStates {
      STOWED,
      IDLE,
      TRACKING_TARGET,
      TRACK_HUB_ON_MOVE,
      TUNING
    }

    public static class TargetPoses {
      public static final Pose3d hub =
          new Pose3d(FieldConstants.Hub.innerCenterPoint, new Rotation3d());
      public static final Pose3d allianceLeft =
          new Pose3d(new Translation3d(FieldConstants.LinesVertical.allianceZone/2,3*(FieldConstants.LinesHorizontal.center/2),1), new Rotation3d());
      public static final Pose3d allianceRight =
          new Pose3d(new Translation3d(FieldConstants.LinesVertical.allianceZone/2,(FieldConstants.LinesHorizontal.center/2),1), new Rotation3d());
    }

  public static final class Turret {

    public static TalonFXSubsystemConfig config = new TalonFXSubsystemConfig();

    public static Angle zeroAngleOffset = Degrees.of(-90);

    static {
      config.name = "Turret";
      config.tunable = true;  // Enable tuning for this subsystem
      config.talonCANID = new CANDeviceId(20); // Example CAN ID, replace with actual ID
      
      config.fxConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      config.fxConfig.Slot0.kS = 0.01;
      config.fxConfig.Slot0.kV = 0.0;//0.12;
      config.fxConfig.Slot0.kA = 0.0;//0.01;
      config.fxConfig.Slot0.kP = 0.0; // 6.4
      config.fxConfig.Slot0.kI = 0.0;
      config.fxConfig.Slot0.kD = 0.0;
      config.unitToRotorRatio =  28.5; // 28.5 to 1

      config.momentOfInertia = MoiUnits.PoundSquareInches.of(500);

      // Motion Magic parameters
      config.fxConfig.MotionMagic.MotionMagicCruiseVelocity = 100.0; // rotations per second
      config.fxConfig.MotionMagic.MotionMagicAcceleration = 100.0; // rotations per second^2
      //config.fxConfig.MotionMagic.MotionMagicJerk = 100; // limit jerk for smooth motion

      config.initialTransform =
          new Transform3d(
              new Translation3d( Inches.of(-4.371627).in(Meters), Inches.of(6.257212).in(Meters), Inches.of(14.008475).in(Meters)),
              new Rotation3d(0, 0, Units.degreesToRadians(zeroAngleOffset.in(Degrees))));        

    } 

    public static int MODEL_INDEX = 3;
    public static int PARENT_INDEX = 0; // drivetrain
    

  } // End of Turret Constants

  public final class Flywheels {

    public static enum FlywheelsState {
      IDLE,
      PRESPIN,
      TESTING_ENABLED,
      SHOOT
    }

    public static TalonFXSubsystemConfig leftConfig = new TalonFXSubsystemConfig();
    public static TalonFXSubsystemConfig rightConfig = new TalonFXSubsystemConfig();

    static {
      leftConfig.name = "Flywheels Left Lead";
      leftConfig.tunable = false;  // Enable tuning for this subsystem
      leftConfig.talonCANID = new CANDeviceId(21); // Example CAN ID, replace with actual ID
      leftConfig.fxConfig.Slot0.kP = 0.0;
      leftConfig.fxConfig.Slot0.kI = 0.0;
      leftConfig.fxConfig.Slot0.kD = 0.0;
      leftConfig.fxConfig.Slot0.kS = 0.15; //0.15
      leftConfig.fxConfig.Slot0.kV = 0.1; //0.114
      leftConfig.unitToRotorRatio = 1.0; // 1:1 ratio
      leftConfig.fxConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
      leftConfig.fxConfig.MotorOutput.PeakReverseDutyCycle = 0;
      //leftConfig.tunable = true;

      rightConfig.name = "Flywheels Right Follower";
      rightConfig.talonCANID = new CANDeviceId(22); // Example CAN ID, replace with actual ID
      rightConfig.unitToRotorRatio = 1.0; // 1:1 ratio
    }

    public static int MODEL_INDEX = 5;
    public static int PARENT_INDEX = 4;

    public static Transform3d localTransform =
        new Transform3d(
            new Translation3d(Inches.of(-5).in(Meters), 0, Inches.of(2).in(Meters)),
            new Rotation3d(0, Degrees.of(-90).in(Radians), 0));

    public static InterpolatingDoubleTreeMap velocityMap = new InterpolatingDoubleTreeMap();

    static {
      // Distance (m) -> Ball Velocity (ft/s)
      velocityMap.put(1.0, 20.0);
      velocityMap.put(1.5, 20.0);
      velocityMap.put(3.0, 23.0);
      velocityMap.put(4.0, 25.0);
      velocityMap.put(5.17, 28.0);
    }
  } // End of Flywheels Constants

  public final class Hood {

    public static TalonFXSubsystemConfig config = new TalonFXSubsystemConfig();
    public static Angle zeroAngleOffset = Degrees.of(18); 
    public static Angle minimumAngle = zeroAngleOffset;
    public static Angle maximumAngle = Degrees.of(18+28); 

    static {
      config.name = "Hood";
      config.tunable = false;  // Enable tuning for this subsystem
      config.talonCANID = new CANDeviceId(23); // Example CAN ID, replace with actual ID
      config.fxConfig.Slot0.kS = 0.0;
      config.fxConfig.Slot0.kP = 0.0;
      config.fxConfig.Slot0.kI = 0.0;
      config.fxConfig.Slot0.kD = 0.0;
      config.unitToRotorRatio = 1.0/102.0; //102 motor rotations would do 1 rotation of the herringbone if it were a big gear

      config.momentOfInertia = MoiUnits.PoundSquareInches.of(100);

      // Motion Magic parameters
      config.fxConfig.MotionMagic.MotionMagicCruiseVelocity = 30.0; // rotations per second
      config.fxConfig.MotionMagic.MotionMagicAcceleration = 60.0; // rotations per second^2
      //config.fxConfig.MotionMagic.MotionMagicJerk = 100; // limit jerk for smooth motion

      config.initialTransform =
          new Transform3d(
              new Translation3d(Inches.of(3.75).in(Meters), 0, Inches.of(1.75).in(Meters)), new Rotation3d());
    }

    public static Angle retractedPosition = minimumAngle;
    public static int MODEL_INDEX = 4;
    public static int PARENT_INDEX = 3; // turret

    public static InterpolatingDoubleTreeMap angleMap = new InterpolatingDoubleTreeMap();

    static {
      // Distance (m) -> Hood Pitch (Degrees) - Example from Redhawk
      angleMap.put(1.0, 15.0);
      angleMap.put(1.5, 22.0);
      angleMap.put(3.0, 30.0);
      angleMap.put(4.0, 40.0);

      // angleMap.put(1.0, 15.0);
      // angleMap.put(4.0, 30.0);
    }


  } // End of Hood Constants

   public static LoggedTunableMeasure<Time> otfLinearProjectionSeconds =
      new LoggedTunableMeasure<Time>(
          "LaunchingSolutionManager/time_to_project_lin", Seconds.of(0.5));
  public static LoggedTunableMeasure<Time> otfAngularProjectionSeconds =
      new LoggedTunableMeasure<Time>(
          "LaunchingSolutionManager/time_to_project_ang", Seconds.of(0.5));
  public static LoggedTunableBoolean otfFutureProjectionEnabled =
      new LoggedTunableBoolean("LaunchingSolutionManager/projection_enabled", true);
}
