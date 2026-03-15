package frc319.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc319.lib.io.ArticulatedComponent;
import frc319.lib.io.TalonFXIO;
import frc319.lib.subsystem.MotorSubsystem;
import frc319.lib.subsystem.TalonFXSubsystemConfig;
import frc319.lib.io.MotorInputsAutoLogged;
import frc319.robot.subsystems.intake.IntakeConstants.IntakeStates;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.Idle;

public class IntakeExtension extends MotorSubsystem<MotorInputsAutoLogged, TalonFXIO>
    implements ArticulatedComponent {

      IntakeStates intakeState = IntakeStates.IDLE;
      boolean shouldExtend = false;
      

      private Distance intakePosition = Inches.of(0);

  public IntakeExtension(
      final TalonFXSubsystemConfig config, final TalonFXIO intakeExtensionMotorIO) {
    super(config, new MotorInputsAutoLogged(), intakeExtensionMotorIO);
  }

  public void setState(IntakeStates state){
    this.intakeState = state;
  }

  /**
   * Move to specified distance with motion magic
   *
   * @param desiredDistance
   * @return
   */
  public Command setDistanceCommand(Supplier<Distance> desiredDistance) {
    return motionMagicSetpointCommand(
        () -> convertSubsystemPositionToMotorPosition(desiredDistance.get()));
  }

  /**
   * Move to the extended position with motion magic
   *
   * @return
   */
  public Command extendCommand() {
    return setDistanceCommand(() -> IntakeConstants.Extension.extendedPosition);
  }

  /**
   * Move to the retracted postion with motion magic
   *
   * @return
   */
  public Command retractCommand() {
    return setDistanceCommand(() -> IntakeConstants.Extension.flushPosition);
  }

  @Override
  public void periodic() {
    Logger.recordOutput(pb.makePath("state"), intakeState);

    super.periodic();

    switch(intakeState){
      case EXTENDED:
        CommandScheduler.getInstance().schedule(this.extendCommand());
        break;

      case FLUSH:
        CommandScheduler.getInstance().schedule(this.setDistanceCommand(() -> IntakeConstants.Extension.flushPosition));
        break;

      case RETRACTED:
        CommandScheduler.getInstance().schedule(this.retractCommand());
        break;

      case TUNING:
        // Implement tuning state behavior here
        intakePosition = Inches.of(SmartDashboard.getNumber("Tuning_Mode/Intake_Tuning_Position_Inches", 0.0));
        CommandScheduler.getInstance().schedule(this.setDistanceCommand(() -> intakePosition));
        break;

      case JOSTLE:

      if(this.isMotionMagicAtTarget() )
      {
        if(getCurrentPositionAsDistance().gt(IntakeConstants.Extension.extendedPosition.minus(Inches.of(2.0)))){
          CommandScheduler.getInstance().schedule(this.setDistanceCommand(() -> IntakeConstants.Extension.flushPosition));
        }
        else
        {
          CommandScheduler.getInstance().schedule(this.setDistanceCommand(() -> IntakeConstants.Extension.extendedPosition));
        }


      }

        // if(getCurrentPositionAsDistance().gt(IntakeConstants.Extension.extendedPosition.minus(Inches.of(0.5)))) {
        //   shouldExtend = false;
        // }
        // else {
        //   shouldExtend = true;
        // }

        // if(shouldExtend){
        //   intakePosition = intakePosition.plus(Inches.of(0.25));
        // }
        // else
        // {
        //   intakePosition = intakePosition.minus(Inches.of(0.25));
        // }
        
        

        break;

      case IDLE:
      default:
        //super.stop();
        break;
    }
  }

  @Override
  public Transform3d getTransform3d() {
    // TODO: Get this from sensors
    //Distance distance = Inches.of(Math.sin(Timer.getFPGATimestamp()) + 1).times(6);

    Distance distance = getCurrentPositionAsDistance();

    Logger.recordOutput(pb.makePath("currentDistanceInches"), distance.in(Inches));

    //TODO - i probably need to adjust this to represent reality vs some arbitrary motion
    // Clamp the rotations to a reasonable range to prevent the hood from rotating to an impossible distance
    if(distance.in(Meters) > Inches.of(12).in(Meters)){
      distance = Inches.of(12);
    } else if(distance.in(Meters) < Inches.of(0).in(Meters)){
      distance = Inches.of(0);
    }

    Angle sliderAngle = Degrees.of(-12);  // was -4.479515

    Distance distanceX = distance.times(Math.cos(sliderAngle.in(Radians)));
    Distance distanceZ = distance.times(Math.sin(sliderAngle.in(Radians)));

    return new Transform3d(
        new Translation3d(distanceX.in(Meters), 0, distanceZ.in(Meters)), new Rotation3d());
  }
}
