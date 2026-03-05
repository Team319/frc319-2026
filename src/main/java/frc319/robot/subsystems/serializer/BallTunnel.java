package frc319.robot.subsystems.serializer;

import java.util.function.DoubleSupplier;

import frc319.lib.io.TalonFXIO;
import frc319.lib.subsystem.MotorSubsystem;
import frc319.lib.subsystem.TalonFXSubsystemConfig;
import frc319.lib.io.MotorInputsAutoLogged;
import frc319.robot.subsystems.serializer.SerializerConstants.SerializerStates;

public class BallTunnel extends MotorSubsystem<MotorInputsAutoLogged, TalonFXIO> {

  private SerializerStates serializerState = SerializerStates.IDLE;

  public BallTunnel(final TalonFXSubsystemConfig config, final TalonFXIO feederMotorIO) {
    super(config, new MotorInputsAutoLogged(), feederMotorIO);
  }

  public void setState(SerializerStates state){
    this.serializerState = state;
  }

  @Override
  public void periodic() {
    super.periodic();

    switch(serializerState){
      case SHOOT:
          this.setDutyCycle(() -> 1.0);
        break;

      
      case IDLE:
      case JOSTLE:
      default:
        this.stop();
        break;
    }  
  }
public void stop() {
   super.setOpenLoopDutyCycleImpl(0.0); 
}
public void setDutyCycle(DoubleSupplier dutyCycle){
  super.setOpenLoopDutyCycleImpl(dutyCycle.getAsDouble());
}

// /**
//    * Sets the supplier for the test flywheel RPM slider.
//    * Call this from RobotContainer to connect an Elastic slider.
//    * 
//    * @param dutyCycleSupplier A DoubleSupplier that returns the desired RPM from Elastic
//    */
//   public void setTestBallTunnelDutyCycle(DoubleSupplier dutyCycleSupplier) {
//     this.ballTunnelDutyCycle = dutyCycleSupplier;
//   }

}
