package frc319.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.Distance;
import frc319.lib.drivers.CANDeviceId;
import frc319.lib.dynamics.MoiUnits;
import frc319.lib.subsystem.TalonFXSubsystemConfig;

public final class IntakeConstants {

  public enum IntakeStates{
    IDLE,
    RETRACTED,
    EXTENDED,
    COLLECT,
    EJECT,
    JOSTLE, // ? is there a better work...
    TUNING

  }

  public static final class Extension {

    public static TalonFXSubsystemConfig config = new TalonFXSubsystemConfig();

    private static final double gearRatio = 3;

    static {
      config.name = "Intake Extension";
      config.tunable = true;
      config.talonCANID = new CANDeviceId(30); 
      config.fxConfig.Slot0.kS = 0.01;
      config.fxConfig.Slot0.kP = 25.0;
      config.fxConfig.Slot0.kI = 1.0;
      config.fxConfig.Slot0.kD = 0.0;

      // Not perfect but close enough
      config.unitRotationsPerMeter = 39.36; // 1 motor rotation / 1 inch of travel *  1 meter / 39.36 inches per meter...;

      config.momentOfInertia = MoiUnits.PoundSquareInches.of(100);

        // Motion Magic parameters
        config.fxConfig.MotionMagic.MotionMagicCruiseVelocity = 500.0; 
        config.fxConfig.MotionMagic.MotionMagicAcceleration = 500.0; 
    }

    public static Distance extendedPosition = Inches.of(16);
    public static Distance retractedPosition = Inches.of(0);
    public static int MODEL_INDEX = 1;
    public static int PARENT_INDEX = 0; // drivetrain

  }

  public static final class Rollers {

    public static TalonFXSubsystemConfig leadConfig = new TalonFXSubsystemConfig();
    public static TalonFXSubsystemConfig followConfig = new TalonFXSubsystemConfig();
    static {
      leadConfig.name = "Intake Rollers Lead";
      leadConfig.tunable = false;
      leadConfig.talonCANID = new CANDeviceId(31); // Example CAN ID, replace with actual ID
      leadConfig.unitToRotorRatio = 1.0; // 1:1 ratio

      followConfig.name = "Intake Rollers Follow";
      followConfig.tunable = false;
      followConfig.talonCANID = new CANDeviceId(32); // Example CAN ID, replace with actual ID
      followConfig.unitToRotorRatio = 1.0; // 1:1 ratio
      followConfig.fxConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    }

    public static double intakeDutyCycle = 1.0;
    public static double outtakeDutyCycle = -1.0;
  }

  
}
