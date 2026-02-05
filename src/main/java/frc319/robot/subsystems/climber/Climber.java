package frc319.robot.subsystems.climber;

import frc319.redhawk_lib.io.MotorInputsAutoLogged;
import frc319.redhawk_lib.io.TalonFXIO;
import frc319.redhawk_lib.subsystem.MotorSubsystem;
import frc319.redhawk_lib.subsystem.TalonFXSubsystemConfig;

public class Climber extends MotorSubsystem<MotorInputsAutoLogged, TalonFXIO> {

  public Climber(final TalonFXSubsystemConfig config, final TalonFXIO climberMotorIO) {
    super(config, new MotorInputsAutoLogged(), climberMotorIO);
  }

  @Override
  public void periodic() {
    super.periodic();
    // Additional periodic code for climber extension can be added here
  }
}
