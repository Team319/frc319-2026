package frc319.robot.subsystems.serializer;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc319.lib.drivers.CANDeviceId;
import frc319.lib.subsystem.TalonFXSubsystemConfig;

public final class SerializerConstants {

  public enum SerializerStates{
    IDLE,
    JOSTLE,
    SHOOT
  }

  public static final class Spindexer {

    public static TalonFXSubsystemConfig config = new TalonFXSubsystemConfig();

    static {
      config.name = "Spindexer";
      config.tunable = true;
      config.talonCANID = new CANDeviceId(24); // Example CAN ID, replace with actual ID
      config.fxConfig.Slot0.kP = 0.01;
      config.fxConfig.Slot0.kI = 0.0;
      config.fxConfig.Slot0.kD = 0.0;
      config.fxConfig.Slot0.kS = 0.01;
      config.fxConfig.Slot0.kV = 0.1175;
      config.fxConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

      config.fxConfig.CurrentLimits.StatorCurrentLimitEnable = true;
      config.fxConfig.CurrentLimits.StatorCurrentLimit = 35.0;

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
      config.tunable = true;
      config.talonCANID = new CANDeviceId(25); // Example CAN ID, replace with actual ID
      config.fxConfig.Slot0.kP = 0.01;
      config.fxConfig.Slot0.kI = 0.0;
      config.fxConfig.Slot0.kD = 0.0;
      config.fxConfig.Slot0.kS = 0.01;
      config.fxConfig.Slot0.kV = 0.1175;

      config.fxConfig.CurrentLimits.StatorCurrentLimitEnable = true;
      config.fxConfig.CurrentLimits.StatorCurrentLimit = 35.0;

      config.unitToRotorRatio = 1.0; // 1:1 ratio
    }
  }
}
