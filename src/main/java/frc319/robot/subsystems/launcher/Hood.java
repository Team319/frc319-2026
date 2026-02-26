package frc319.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc319.redhawk_lib.io.ArticulatedComponent;
import frc319.redhawk_lib.io.MotorInputsAutoLogged;
import frc319.redhawk_lib.io.TalonFXIO;
import frc319.redhawk_lib.subsystem.MotorSubsystem;
import frc319.redhawk_lib.subsystem.TalonFXSubsystemConfig;
import frc319.robot.FieldConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Hood extends MotorSubsystem<MotorInputsAutoLogged, TalonFXIO>
    implements ArticulatedComponent {

  private LauncherConstants.LauncherStates laucherState = LauncherConstants.LauncherStates.IDLE;

  Angle aimAngle = Degrees.of(0.0);
  Distance toGoal = Meters.of(0.0);

  public Hood(final TalonFXSubsystemConfig config, final TalonFXIO launcherMotorIO) {
    super(config, new MotorInputsAutoLogged(), launcherMotorIO);
    setMotionMagicConfigImpl(LauncherConstants.Hood.mmConfig);

  }

  public void setState(LauncherConstants.LauncherStates state) {
    this.laucherState = state;
  }

  public Command setAngle(Supplier<Angle> desiredAngle) {
    return motionMagicSetpointCommand(
        () -> convertSubsystemPositionToMotorPosition(desiredAngle.get()));
  }

  public Command retract() {
    return setAngle(() -> LauncherConstants.Hood.retractedPosition);
  }

  @Override
  public void periodic() {
    Logger.recordOutput(pb.makePath("state"), laucherState);
    super.periodic();

    switch (laucherState) {
      case STOWED:
          aimAngle = Degrees.of(0.0);
          Logger.recordOutput(super.pb.makePath("aimAngle"), aimAngle);
          this.retract().schedule();
        break;

      case TRACKING_TARGET:
          toGoal = this.getDistance2d(LaunchingSolutionManager.getInstance().getTargetPose());
          Logger.recordOutput(super.pb.makePath("distanceToGoal"), toGoal);
          aimAngle = Degrees.of(LauncherConstants.Hood.angleMap.get(toGoal.in(Meters)));
          Logger.recordOutput(super.pb.makePath("aimAngle"), aimAngle);
          this.setAngle(()->aimAngle).schedule();
      
      case TRACK_HUB_ON_MOVE:
          // Calculate angle to adjust when Shooting on the Move
          toGoal = this.getDistance2d(LaunchingSolutionManager.getInstance().getTargetOnMovePose());
          Logger.recordOutput(super.pb.makePath("distanceToGoal"), toGoal);
          aimAngle = Degrees.of(LauncherConstants.Hood.angleMap.get(toGoal.in(Meters)));
          Logger.recordOutput(super.pb.makePath("aimAngle"), aimAngle);
          this.setAngle(()->aimAngle).schedule();
        break;

      case IDLE:
        super.stop();
      default:
        break;
    }

  }

  @Override
  public Transform3d getTransform3d() {
    Angle rotations = super.getCurrentPosition().times(config.unitToRotorRatio);
    Transform3d localTransform =
        new Transform3d(new Translation3d(), new Rotation3d(0, rotations.in(Radians), 0));

    return config.initialTransform.plus(localTransform);
  }

  @Override
  public Translation3d getRelativeAngularVelocity() {
    return new Translation3d(0, super.getCurrentVelocity().in(RadiansPerSecond), 0);
  }
}
