package frc319.robot.subsystems.serializer;

import java.util.function.DoubleSupplier;

import frc319.redhawk_lib.io.MotorInputsAutoLogged;
import frc319.redhawk_lib.io.TalonFXIO;
import frc319.redhawk_lib.subsystem.MotorSubsystem;
import frc319.redhawk_lib.subsystem.TalonFXSubsystemConfig;

public class BallTunnel extends MotorSubsystem<MotorInputsAutoLogged, TalonFXIO> {

  DoubleSupplier ballTunnelDutyCycle;
  public BallTunnel(final TalonFXSubsystemConfig config, final TalonFXIO feederMotorIO) {
    super(config, new MotorInputsAutoLogged(), feederMotorIO);
  }

  @Override
  public void periodic() {
    super.periodic();
    setDutyCycle(ballTunnelDutyCycle);
    // Additional periodic code for feeder can be added here
  }
public void stop() {
   super.setOpenLoopDutyCycleImpl(0.0); 
}
public void setDutyCycle(DoubleSupplier dutyCycle){
  super.setOpenLoopDutyCycleImpl(dutyCycle.getAsDouble());
}
/**
   * Sets the supplier for the test flywheel RPM slider.
   * Call this from RobotContainer to connect an Elastic slider.
   * 
   * @param dutyCycleSupplier A DoubleSupplier that returns the desired RPM from Elastic
   */
  public void setTestBallTunnelDutyCycle(DoubleSupplier dutyCycleSupplier) {
    this.ballTunnelDutyCycle = dutyCycleSupplier;
  }

}
