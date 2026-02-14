package frc319.robot.subsystems.serializer;

import frc319.redhawk_lib.io.MotorInputsAutoLogged;
import frc319.redhawk_lib.io.TalonFXIO;
import frc319.redhawk_lib.subsystem.MotorSubsystem;
import frc319.redhawk_lib.subsystem.TalonFXSubsystemConfig;

public class BallTunnel extends MotorSubsystem<MotorInputsAutoLogged, TalonFXIO> {

  public BallTunnel(final TalonFXSubsystemConfig config, final TalonFXIO feederMotorIO) {
    super(config, new MotorInputsAutoLogged(), feederMotorIO);
  }

  @Override
  public void periodic() {
    super.periodic();
    // Additional periodic code for feeder can be added here
  }
public void stop() {
   super.setOpenLoopDutyCycleImpl(0.0); 
}
public void setDutyCycle(double dutyCycle){
  super.setOpenLoopDutyCycleImpl(dutyCycle);
}
}
