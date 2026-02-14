package frc319.robot.subsystems.serializer;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import frc319.redhawk_lib.io.ArticulatedComponent;
import frc319.redhawk_lib.io.MotorInputsAutoLogged;
import frc319.redhawk_lib.io.TalonFXIO;
import frc319.redhawk_lib.subsystem.MotorSubsystem;
import frc319.redhawk_lib.subsystem.TalonFXSubsystemConfig;

public class Spindexer extends MotorSubsystem<MotorInputsAutoLogged, TalonFXIO>
    implements ArticulatedComponent {

      DoubleSupplier spindexerDutyCycle;

  public Spindexer(final TalonFXSubsystemConfig config, final TalonFXIO indexerMotorIO) {
    super(config, new MotorInputsAutoLogged(), indexerMotorIO);
  }

  @Override
  public void periodic() {
    super.periodic();
    setDutyCycle(spindexerDutyCycle.getAsDouble());
    // Additional periodic code for indexer can be added here
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
  public void setTestSpindexerDutyCycle(DoubleSupplier dutyCycleSupplier) {
    this.spindexerDutyCycle = dutyCycleSupplier;
  }
}
