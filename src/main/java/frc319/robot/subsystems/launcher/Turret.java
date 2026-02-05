package frc319.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc319.redhawk_lib.io.ArticulatedComponent;
import frc319.redhawk_lib.io.MotorInputsAutoLogged;
import frc319.redhawk_lib.io.TalonFXIO;
import frc319.redhawk_lib.subsystem.KinematicsManager;
import frc319.redhawk_lib.subsystem.MotorSubsystem;
import frc319.redhawk_lib.subsystem.TalonFXSubsystemConfig;
import frc319.redhawk_lib.util.Util;
import frc319.robot.FieldConstants;

import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Turret extends MotorSubsystem<MotorInputsAutoLogged, TalonFXIO>
    implements ArticulatedComponent {

  private LauncherConstants.Turret.TurretState turretState = LauncherConstants.Turret.TurretState.IDLE;

  private Pose3d currentTargetPose = new Pose3d(); // TODO : Blue origin for now. but use center field or something... 

  public Turret(final TalonFXSubsystemConfig config, final TalonFXIO turretMotorIO) {
    super(config, new MotorInputsAutoLogged(), turretMotorIO);
    setMotionMagicConfigImpl(LauncherConstants.Turret.mmConfig);


   
  }

  public void setState(LauncherConstants.Turret.TurretState newState) {
    this.turretState = newState;
  }

  public Command setAngle(Supplier<Angle> desiredAngle) {
      
    return motionMagicSetpointCommand(
        () -> convertSubsystemPositionToMotorPosition( desiredAngle.get() ));
  }

  @Override
  public void periodic() {

    super.periodic();
    
    switch (turretState) {
      case TRACKING_TARGET:
        this.setAngle(() -> getCurrentTargetAngle()).schedule();
        currentTargetPose = LaunchingSolutionManager.getInstance().getTargetPose();
        break;
      case TRACK_HUB_ON_MOVE:
        this.setAngle(() -> getLauncOnTheFlyAngle()).schedule();
        currentTargetPose = LaunchingSolutionManager.getInstance().getTargetOnMovePose();
        break;
      case IDLE:
      default:
      this.setAngle(() -> Degrees.of(0)).schedule();
        break;
    }

    Logger.recordOutput(pb.makePath("goalVector"), new Pose3d[] {this.getGlobalPose(), currentTargetPose});

  };

  @Override
  public Transform3d getTransform3d() {
    Angle rotations = super.getCurrentPosition().times(config.unitToRotorRatio);
    return config.initialTransform.plus(
        new Transform3d(new Translation3d(), new Rotation3d(0, 0, rotations.in(Radians))));
  }

  @Override
  public Translation3d getRelativeAngularVelocity() {
    return new Translation3d(0, 0, super.getCurrentVelocity().in(RadiansPerSecond));
  }

  public Angle getHubAngle() {
    Translation3d diff = this.getTranslationTo(FieldConstants.Hub.topCenterPoint);
    // 2. Calculate the Global Yaw needed to face the target
    // Math.atan2(y, x) handles all quadrants correctly
    double globalTargetRadians = Math.atan2(diff.getY(), diff.getX());

    // 3. Get the Chassis Heading (Global)
    // You need the Robot's orientation on the field to make this relative.
    // Assuming ID 0 is your Drive/Chassis in KinematicsManager:
    Rotation3d chassisRotation = KinematicsManager.getInstance().getGlobalPose(0).getRotation();
    double chassisHeadingRadians = chassisRotation.getZ();

    // 4. Calculate Relative Angle (Target - Chassis)
    double relativeRadians = globalTargetRadians - chassisHeadingRadians;

    // 5. Normalize to range (-PI to PI) so the turret takes the shortest path
    // e.g., if result is 350 degrees, this turns it into -10 degrees
    double normalizedRadians = MathUtil.angleModulus(relativeRadians);

    return Radians.of(normalizedRadians);
  }

  public Angle getCurrentTargetAngle() {
    Translation3d diff = this.getTranslationTo(LaunchingSolutionManager.getInstance().getTargetPose().getTranslation());
    // 2. Calculate the Global Yaw needed to face the target
    // Math.atan2(y, x) handles all quadrants correctly
    double globalTargetRadians = Math.atan2(diff.getY(), diff.getX());

    // 3. Get the Chassis Heading (Global)
    // You need the Robot's orientation on the field to make this relative.
    // Assuming ID 0 is your Drive/Chassis in KinematicsManager:
    Rotation3d chassisRotation = KinematicsManager.getInstance().getGlobalPose(0).getRotation();
    double chassisHeadingRadians = chassisRotation.getZ();

    // 4. Calculate Relative Angle (Target - Chassis)
    double relativeRadians = globalTargetRadians - chassisHeadingRadians;

    // 5. Normalize to range (-PI to PI) so the turret takes the shortest path
    // e.g., if result is 350 degrees, this turns it into -10 degrees
    double normalizedRadians = MathUtil.angleModulus(relativeRadians);

    return Radians.of(normalizedRadians);
  }

  public Angle getLauncOnTheFlyAngle() {
    // 1. Get the latest solution
    var solution = LaunchingSolutionManager.getInstance().getSolution();

    if (solution.isValid()) {
      // 2. Convert Field-Relative Goal to Robot-Relative Setpoint

      // Get Chassis Heading from Kinematics
      Rotation2d chassisHeading =
          KinematicsManager.getInstance().getGlobalPose(0).getRotation().toRotation2d();

      // TargetYaw - ChassisYaw = TurretSetpoint
      Rotation2d localSetpoint = solution.turretFieldRelativeYaw().minus(chassisHeading);

      return localSetpoint.getMeasure();
    }
    return Degrees.of(0);
  }
}
