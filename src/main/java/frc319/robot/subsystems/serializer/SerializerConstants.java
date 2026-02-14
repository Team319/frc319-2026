package frc319.robot.subsystems.serializer;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc319.redhawk_lib.drivers.CANDeviceId;
import frc319.redhawk_lib.subsystem.TalonFXSubsystemConfig;

public final class SerializerConstants {

  public static final class Spindexer {

    public static TalonFXSubsystemConfig config = new TalonFXSubsystemConfig();

    static {
      config.name = "Spindexer";
      config.talonCANID = new CANDeviceId(10); // Example CAN ID, replace with actual ID
      config.fxConfig.Slot0.kP = 0.2;
      config.fxConfig.Slot0.kI = 0.0;
      config.fxConfig.Slot0.kD = 0.0;
      config.unitToRotorRatio = 1.0; // 1:1 ratio
      config.initialTransform =
          new Transform3d(new Translation3d(0, Inches.of(1.75).in(Meters), 0), new Rotation3d());
    }

    public static int MODEL_INDEX = 2;
    public static int PARENT_INDEX = 0; // drivetrain
  }

  public static final class BallTunnel {

    public static TalonFXSubsystemConfig config = new TalonFXSubsystemConfig();

    static {
      config.name = "BallTunnel";
      config.talonCANID = new CANDeviceId(11); // Example CAN ID, replace with actual ID
      config.fxConfig.Slot0.kP = 0.2;
      config.fxConfig.Slot0.kI = 0.0;
      config.fxConfig.Slot0.kD = 0.0;
      config.unitToRotorRatio = 1.0; // 1:1 ratio
    }
  }
}
