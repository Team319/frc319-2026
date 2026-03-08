package frc319.robot.subsystems.serializer;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc319.lib.io.ArticulatedComponent;
import frc319.lib.io.TalonFXIO;
import frc319.lib.subsystem.MotorSubsystem;
import frc319.lib.subsystem.TalonFXSubsystemConfig;
import frc319.lib.io.MotorInputsAutoLogged;
import frc319.robot.subsystems.serializer.SerializerConstants.SerializerStates;

public class Spindexer extends MotorSubsystem<MotorInputsAutoLogged, TalonFXIO>
    implements ArticulatedComponent {


  private SerializerStates serializerState = SerializerStates.IDLE;

  private AngularVelocity spindexerVelocity = RPM.of(0.0);

  public Spindexer(final TalonFXSubsystemConfig config, final TalonFXIO indexerMotorIO) {
    super(config, new MotorInputsAutoLogged(), indexerMotorIO);
  }

  public void setState(SerializerStates state){
    this.serializerState = state;
  }

  public Command setVelocity(Supplier<AngularVelocity> desiredVelocity) {
    return velocitySetpointCommand(desiredVelocity);
  }

  @Override
  public void periodic() {
    Logger.recordOutput(pb.makePath("state"), serializerState);
    Logger.recordOutput(pb.makePath("curretSpindexerRPM"), super.getCurrentVelocity().in(RPM));


    super.periodic();
    
    switch(serializerState){
      case SHOOT:
          //spindexerVelocity = RPM.of(SmartDashboard.getNumber("Tuning_Mode/Spindexer_Tuning_RPM", 0.0));
          spindexerVelocity = RPM.of(6000.0);
          this.setVelocity(()-> spindexerVelocity ).schedule();
        break;

      
      case IDLE:
      case JOSTLE:
      default:
        this.setVelocity(()-> RPM.of(0.0)).schedule();
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
