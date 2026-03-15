package frc319.robot.subsystems.serializer;

import static edu.wpi.first.units.Units.RPM;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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

  public Command setVelocity(Supplier<AngularVelocity> desiredVelocity) {
    return velocitySetpointCommand(desiredVelocity);
  }

  @Override
  public void periodic() {
    super.periodic();
        Logger.recordOutput(pb.makePath("curretBallTunnelRPM"), super.getCurrentVelocity().in(RPM));


    switch(serializerState){
      case SHOOT:            
          CommandScheduler.getInstance().schedule(this.velocitySetpointCommand(() -> RPM.of( 6000.0))); // TODO :  need matching ball tunnel ANGULAR Velocity --- IGNORE ---
        break;

      
      case IDLE:
      case JOSTLE:
      default:
        CommandScheduler.getInstance().schedule(this.velocitySetpointCommand(() -> RPM.of( 0.0)));

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
