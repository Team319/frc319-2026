package frc319.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc319.lib.io.ArticulatedComponent;
import frc319.lib.io.TalonFXIO;
import frc319.lib.subsystem.MotorSubsystem;
import frc319.lib.subsystem.TalonFXSubsystemConfig;
import frc319.lib.util.EqualsUtil;
import frc319.lib.io.MotorInputsAutoLogged;
import frc319.robot.FieldConstants;
import frc319.robot.subsystems.launcher.LaunchingSolutionManager.LaunchSolution;

import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Hood extends MotorSubsystem<MotorInputsAutoLogged, TalonFXIO>
    implements ArticulatedComponent {

  private LauncherConstants.LauncherStates laucherState = LauncherConstants.LauncherStates.IDLE;

  Angle aimAngle = Degrees.of(0.0);
  Distance toGoal = Meters.of(0.0);

  public Hood(final TalonFXSubsystemConfig config, final TalonFXIO launcherMotorIO) {
    super(config, new MotorInputsAutoLogged(), launcherMotorIO);
    //super.setCurrentPosition(LauncherConstants.Hood.zeroAngleOffset);

  }

  public void setState(LauncherConstants.LauncherStates state) {
    this.laucherState = state;
  }

  public Command setAngle(Supplier<Angle> desiredAngle) {
    return motionMagicSetpointCommand(
        () -> convertSubsystemPositionToMotorPosition(desiredAngle.get()));
        
  }

  @Override
  protected Angle convertSubsystemPositionToMotorPosition(Angle subsystemPosition) {
    return subsystemPosition.divide(config.unitToRotorRatio);
  }

  protected Angle convertMotorPositionToSubsystemPosition(Angle motorPosition) {
    return motorPosition.times(config.unitToRotorRatio);
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
          break;
      
      case TRACK_HUB_ON_MOVE:
          // Calculate angle to adjust when Shooting on the Move
          toGoal = this.getDistance2d(LaunchingSolutionManager.getInstance().getTargetOnMovePose());
          Logger.recordOutput(super.pb.makePath("distanceToGoal"), toGoal);
          aimAngle = Degrees.of(LauncherConstants.Hood.angleMap.get(toGoal.in(Meters)));
          Logger.recordOutput(super.pb.makePath("aimAngle"), aimAngle);
          this.setAngle(()->aimAngle).schedule();
        break;

      case TUNING:
        aimAngle = Degrees.of(SmartDashboard.getNumber("Tuning_Mode/Hood_Tuning_Position_Degrees", 0.0));
        this.setAngle(()->aimAngle).schedule();
        break;

      case IDLE:
        //super.stop();
      default:
        break;
    }

  }

  @Override
  public Transform3d getTransform3d() {
    Angle hoodAngle = this.convertMotorPositionToSubsystemPosition(super.getCurrentPosition());
    Logger.recordOutput(super.pb.makePath("hoodAngle_Degrees"), hoodAngle.in(Degrees));

    //TODO - i probably need to adjust this to represent reality vs some arbitrary motion
    // Clamp the hoodAngle to a reasonable range to prevent the hood from rotating to an impossible angle 
    if(hoodAngle.in(Radians) > LauncherConstants.Hood.maximumAngle.in(Radians)){
      hoodAngle = LauncherConstants.Hood.maximumAngle;
    } else if(hoodAngle.in(Radians) < LauncherConstants.Hood.minimumAngle.in(Radians)){
      hoodAngle = LauncherConstants.Hood.minimumAngle;
    }

    Transform3d localTransform =
        new Transform3d(new Translation3d(), new Rotation3d(0, hoodAngle.in(Radians), 0));

    return config.initialTransform.plus(localTransform);
  }

  @Override
  public Translation3d getRelativeAngularVelocity() {
    return new Translation3d(0, super.getCurrentVelocity().in(RadiansPerSecond), 0);
  }

    public boolean isAtTargetPosition() {
    Angle currentAngle = super.getCurrentPosition().times(config.unitToRotorRatio);
    Angle targetAngle = Degrees.of(0.0); // Default to current aimAngle, will be updated based on state

    switch (laucherState) {

      case TRACKING_TARGET:      
      case TRACK_HUB_ON_MOVE:
       targetAngle = aimAngle;

        break;

      default:
        return false; // If we're not actively tracking, we can consider ourselves "at target"
    }

    return EqualsUtil.epsilonEquals(currentAngle.in(Degrees), targetAngle.in(Degrees), 2.0); 
  }
}
