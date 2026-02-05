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

  public Hood(final TalonFXSubsystemConfig config, final TalonFXIO launcherMotorIO) {
    super(config, new MotorInputsAutoLogged(), launcherMotorIO);
  }

  public Command setAngleCommand(Supplier<Angle> desiredAngle) {
    return motionMagicSetpointCommand(
        () -> convertSubsystemPositionToMotorPosition(desiredAngle.get()));
  }

  public Command retract() {
    return setAngleCommand(() -> LauncherConstants.Hood.retractedPosition);
  }

  @Override
  public void periodic() {
    super.periodic();
    Distance toGoal = this.getDistance2d(FieldConstants.Hub.topCenterPoint);
    Logger.recordOutput(super.pb.makePath("distanceToGoal"), toGoal);
    Angle aimAngle = Degrees.of(LauncherConstants.Hood.angleMap.get(toGoal.in(Meters)));

    Logger.recordOutput(super.pb.makePath("aimAngle"), aimAngle);
    super.setCurrentPosition(aimAngle);
  }

  @Override
  public Transform3d getTransform3d() {
    // TODO: Get this from sensors
    Angle rotations = super.getCurrentPosition().div(config.unitToRotorRatio);
    Transform3d localTransform =
        new Transform3d(new Translation3d(), new Rotation3d(0, rotations.in(Radians), 0));

    return config.initialTransform.plus(localTransform);
  }

  @Override
  public Translation3d getRelativeAngularVelocity() {
    return new Translation3d(0, super.getCurrentVelocity().in(RadiansPerSecond), 0);
  }
}
