package frc319.robot.subsystems.serializer;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import frc319.lib.io.ArticulatedComponent;
import frc319.lib.io.TalonFXIO;
import frc319.lib.subsystem.MotorSubsystem;
import frc319.lib.subsystem.TalonFXSubsystemConfig;
import frc319.lib.io.MotorInputsAutoLogged;
import frc319.robot.subsystems.serializer.SerializerConstants.SerializerStates;

public class Spindexer extends MotorSubsystem<MotorInputsAutoLogged, TalonFXIO>
    implements ArticulatedComponent {


  private SerializerStates serializerState = SerializerStates.IDLE;

  public Spindexer(final TalonFXSubsystemConfig config, final TalonFXIO indexerMotorIO) {
    super(config, new MotorInputsAutoLogged(), indexerMotorIO);
  }

  public void setState(SerializerStates state){
    this.serializerState = state;
  }

  @Override
  public void periodic() {
    Logger.recordOutput(pb.makePath("state"), serializerState);

    super.periodic();
    
    switch(serializerState){
      case SHOOT:
          this.setDutyCycle(1.0);
        break;

      
      case IDLE:
      case JOSTLE:
      default:
        //super.stop();
        break;
    }
  }

  @Override
  public Transform3d getTransform3d() {
    // TODO: Get this from sensors
    Angle rotations = Rotations.of(Math.sin(Timer.getFPGATimestamp()) - 1);
    Transform3d localTransform =
        new Transform3d(new Translation3d(), new Rotation3d(0, 0, rotations.in(Radians)));

    return config.initialTransform.plus(localTransform);
  }

  public void stop(){
    super.setOpenLoopDutyCycleImpl(0.0);
  }

  public void setDutyCycle(double dutyCycle){
    super.setOpenLoopDutyCycleImpl(dutyCycle);
  }

  /**
   * Sets the supplier for the test flywheel RPM slider.
   * Call this from RobotContainer to connect an Elastic slider.
   * 
   * @param dutyCycleSupplier A DoubleSupplier that returns the desired RPM from Elastic
   */
  // public void setTestSpindexerDutyCycle(DoubleSupplier dutyCycleSupplier) {
  //   this.spindexerDutyCycle = dutyCycleSupplier;
  // }
}
