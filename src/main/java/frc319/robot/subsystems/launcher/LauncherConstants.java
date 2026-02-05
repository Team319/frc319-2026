package frc319.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import frc319.redhawk_lib.drivers.CANDeviceId;
import frc319.redhawk_lib.subsystem.TalonFXSubsystemConfig;
import frc319.robot.Constants;

public final class LauncherConstants {

  public static final class Turret {

    public static enum TurretState {
      IDLE,
      TRACKING_TARGET,
      TRACK_HUB_ON_MOVE
    }

    public static TalonFXSubsystemConfig config = new TalonFXSubsystemConfig();

    static {
      config.name = "Turret";
      config.talonCANID = new CANDeviceId(20); // Example CAN ID, replace with actual ID
      
      config.fxConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      config.fxConfig.Slot0.kS = 0.4;
      config.fxConfig.Slot0.kV = 0.0;
      config.fxConfig.Slot0.kA = 0.0;
      config.fxConfig.Slot0.kP = 10.0; // 6.4
      config.fxConfig.Slot0.kI = 0.0;
      config.fxConfig.Slot0.kD = 0.0;
      config.unitToRotorRatio = 0.08695;// = 1/(( 92*3 )/ 24)    // 1/(250/24) = 0.0959;

      switch (Constants.getRobot()) {
        case DEVBOT:
            config.initialTransform =
              new Transform3d(
                  new Translation3d(0, Inches.of(0).in(Meters), Inches.of(18.484119).in(Meters)),
                  new Rotation3d(0, 0, Units.degreesToRadians(0)));
          break;

        // Real Robot Geometry
        case COMPBOT:
        case SIMBOT:
        default:
          config.initialTransform =
              new Transform3d(
                  new Translation3d(Inches.of(-9).in(Meters), Inches.of(12).in(Meters), Inches.of(18.484119).in(Meters)),
                  new Rotation3d(0, 0, Units.degreesToRadians(0)));
          break;
        

      }

    }

    public static MotionMagicConfigs mmConfig = new MotionMagicConfigs();

    static {
      mmConfig.MotionMagicCruiseVelocity = 500.0;
      mmConfig.MotionMagicAcceleration = 500.0;
    }

    public static int MODEL_INDEX = 3;
    public static int PARENT_INDEX = 0; // drivetrain
  }

  public final class Flywheels {

    public static TalonFXSubsystemConfig config = new TalonFXSubsystemConfig();

    static {
      config.name = "Flywheels";
      config.talonCANID = new CANDeviceId(13); // Example CAN ID, replace with actual ID
      config.fxConfig.Slot0.kP = 0.2;
      config.fxConfig.Slot0.kI = 0.0;
      config.fxConfig.Slot0.kD = 0.0;
      config.unitToRotorRatio = 1.0; // 1:1 ratio
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
  }

  public final class Hood {

    public static TalonFXSubsystemConfig config = new TalonFXSubsystemConfig();

    static {
      config.name = "Hood";
      config.talonCANID = new CANDeviceId(14); // Example CAN ID, replace with actual ID
      config.fxConfig.Slot0.kP = 0.2;
      config.fxConfig.Slot0.kI = 0.0;
      config.fxConfig.Slot0.kD = 0.0;
      config.unitToRotorRatio = 1.0; // 1:1 ratio
      config.initialTransform =
          new Transform3d(
              new Translation3d(Inches.of(4.086915).in(Meters), 0, 0), new Rotation3d());
    }

    public static Angle retractedPosition = Degrees.of(0);
    public static int MODEL_INDEX = 4;
    public static int PARENT_INDEX = 3; // turret

    public static InterpolatingDoubleTreeMap angleMap = new InterpolatingDoubleTreeMap();

    static {
      // Distance (m) -> Hood Pitch (Degrees)
      angleMap.put(1.0, 15.0);
      angleMap.put(1.5, 22.0);
      angleMap.put(3.0, 30.0);
      angleMap.put(4.0, 40.0);
    }
  }
}
