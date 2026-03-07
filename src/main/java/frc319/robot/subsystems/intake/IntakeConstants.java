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
      config.tunable = false;
      config.talonCANID = new CANDeviceId(30); 
      config.fxConfig.Slot0.kS = 0.01;
      config.fxConfig.Slot0.kP = 0.0;
      config.fxConfig.Slot0.kI = 0.0;
      config.fxConfig.Slot0.kD = 0.0;

      
      config.unitToRotorRatio = 1/gearRatio; 
                                      // TODO - is this the gearRatio or the inverse of the gear ratio 
                                      // 6/25 reduction.
                                      //    (3/1) 1/1 1/1 1/1 lantern is 6/25, so 25/6 = 4.1667, but then we have a 3:1 sprocket reduction after that, so 4.1667*3 = 12.5.
                                      //     // counting ... 
                                      // 12.5 motor rations to travel 12 inches

                                      // diameter of lantern gear is 1.25 inches
      
      config.unitRotationsPerMeter = gearRatio * ((Inches.of(1.25).times(Math.PI).in(Meters)));

      config.momentOfInertia = MoiUnits.PoundSquareInches.of(100);

        // Motion Magic parameters
        config.fxConfig.MotionMagic.MotionMagicCruiseVelocity = 1.0; // meters per second
        config.fxConfig.MotionMagic.MotionMagicAcceleration = 2.0; // meters per second^2
    }

    public static Distance extendedPosition = Inches.of(12);
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
