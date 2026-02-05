package frc319.robot.subsystems.climber;

import frc319.redhawk_lib.drivers.CANDeviceId;
import frc319.redhawk_lib.subsystem.TalonFXSubsystemConfig;

public final class ClimberConstants {

  public static TalonFXSubsystemConfig config = new TalonFXSubsystemConfig();

  static {
    config.name = "Climber";
    config.talonCANID = new CANDeviceId(15); // Example CAN ID, replace with actual ID
    config.fxConfig.Slot0.kP = 0.2;
    config.fxConfig.Slot0.kI = 0.0;
    config.fxConfig.Slot0.kD = 0.0;
    config.unitToRotorRatio = 1.0; // 1:1 ratio
  }
}
