package frc319.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc319.lib.geometry.GeometryUtil;
import frc319.lib.io.ArticulatedComponent;
import frc319.lib.io.TalonFXIO;
import frc319.lib.subsystem.KinematicsManager;
import frc319.lib.subsystem.MotorSubsystem;
import frc319.lib.subsystem.TalonFXSubsystemConfig;
import frc319.lib.util.EqualsUtil;
import frc319.lib.util.Util;
import frc319.lib.io.MotorInputsAutoLogged;
import frc319.robot.Constants;
import frc319.robot.FieldConstants;
import frc319.robot.subsystems.launcher.LaunchingSolutionManager.LaunchSolution;

import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Turret extends MotorSubsystem<MotorInputsAutoLogged, TalonFXIO>
    implements ArticulatedComponent {

  private LauncherConstants.LauncherStates laucherState = LauncherConstants.LauncherStates.IDLE;
  private Angle manualNudgeAngle = Degrees.of(0.0);

  private Pose3d currentTargetPose = new Pose3d(); // TODO : Blue origin for now. but use center field or something... 
  private Angle tuningAngle = Degrees.of(0.0);

  private Angle targetAngle = Degrees.of(0.0);

  public Turret(final TalonFXSubsystemConfig config, final TalonFXIO turretMotorIO) {
    super(config, new MotorInputsAutoLogged(), turretMotorIO);
    //setMotionMagicConfigImpl(LauncherConstants.Turret.config.fxConfig.MotionMagic);
  }

  public void setState(LauncherConstants.LauncherStates state) {
    this.laucherState = state;
  }

  public void updateManualNudgeAngle(Angle angle) {
    this.manualNudgeAngle = this.manualNudgeAngle.plus(angle);
  }

  public Angle convertToClosestBoundedTurretAngleDegrees(Angle desiredAngle, Angle current) {
    // Normalize target to [-180, 180] first
    Angle normalizedTarget =
        GeometryUtil.angleModulus(desiredAngle, LauncherConstants.Turret.reverseSoftLimit, LauncherConstants.Turret.forwardSoftLimit);

    // Calculate the shortest path to the target (normalized to [-180, 180])

    Angle diff =
        GeometryUtil.angleModulus(
            normalizedTarget.minus(current), LauncherConstants.Turret.reverseSoftLimit, LauncherConstants.Turret.forwardSoftLimit);

    // Calculate the final absolute position
    Angle finalPosition = normalizedTarget;//current.plus(diff);

    Angle theoreticaAngle = finalPosition;

    // Check if final position is within limits, if not, try the other way around
    if (finalPosition.gt(LauncherConstants.Turret.forwardSoftLimit)) {
      finalPosition = LauncherConstants.Turret.forwardSoftLimit;//
      theoreticaAngle = finalPosition.minus(Rotations.of(1));
    } else if (finalPosition.lt(LauncherConstants.Turret.reverseSoftLimit)) {
      finalPosition = LauncherConstants.Turret.reverseSoftLimit;//
      theoreticaAngle = finalPosition.plus(Rotations.of(1));
    }

    Logger.recordOutput(pb.makePath("setpoint", "theoretical_boundedAngle"), theoreticaAngle.in(Degrees));

    //Angle convertedPosition = convertSubsystemPositionToMotorPosition(finalPosition);

    return theoreticaAngle;
  }

  /** Input should be robot relative (i.e. encoder-reported angle) */
  public Command setAngle(Supplier<Angle> desiredAngle) {
    return motionMagicSetpointCommand(
        () -> {

          // Apply manual nudge angle
          Angle nudgedAngle = desiredAngle.get().plus(manualNudgeAngle);

          // Convert the desired angle to a bounded angle that respects turret limits
          Angle boundedAngleDegrees =
              convertToClosestBoundedTurretAngleDegrees(nudgedAngle, inputs.position);
          Logger.recordOutput(
              pb.makePath("setpoint", "commandedAngle"), desiredAngle.get().in(Degrees));
          Logger.recordOutput(pb.makePath("setpoint", "boundedAngle"), boundedAngleDegrees.in(Degrees));

          Angle finalConvertedAngle = convertSubsystemPositionToMotorPosition(boundedAngleDegrees);
          Logger.recordOutput(pb.makePath("setpoint", "boundedAngle"), boundedAngleDegrees.in(Degrees));
          Logger.recordOutput(pb.makePath("setpoint", "finalConvertedAngle"), finalConvertedAngle.in(Degrees));

          return finalConvertedAngle;//boundedAngleDegrees;
        });
  }

  // public Command setAngle(Supplier<Angle> desiredAngle) {
  //   return motionMagicSetpointCommand(
  //       () -> convertSubsystemPositionToMotorPosition( desiredAngle.get()/* .minus(LauncherConstants.Turret.zeroAngleOffset)*/ ));
  // }

  @Override
  public void periodic() {
    Logger.recordOutput(pb.makePath("state"), laucherState);

    super.periodic();
    
    switch (laucherState) {

      // case TRACKING_TARGET:
      //   CommandScheduler.getInstance().schedule(this.setAngle(() -> getCurrentTargetAngleWithVisionCorrection()));
      //   currentTargetPose = LaunchingSolutionManager.getInstance().getTargetPose();
      //   Logger.recordOutput(super.pb.makePath("currentTargetPose"), currentTargetPose);
      //   break;

      case TRACKING_TARGET:
        // if (Constants.useTurretLimelight && LauncherVisionManager.getInstance().isTargetVisible()) { // add leniency if the last vision measurement was recent fpga timestamp
        //     CommandScheduler.getInstance().schedule(this.setAngle(() -> getTurretAngleFromVision()));
        //     targetAngle = getTurretAngleFromVision();
        // } else {
            // Fall back to drive base pose until LL acquires
            CommandScheduler.getInstance().schedule(this.setAngle(() -> getCurrentTargetAngle()));
            targetAngle = getCurrentTargetAngle();
        // }
        break;
      
      case TRACK_HUB_ON_MOVE:
        targetAngle = getLauncOnTheFlyAngle();
        CommandScheduler.getInstance().schedule(this.setAngle(() -> getLauncOnTheFlyAngle()));
        currentTargetPose = LaunchingSolutionManager.getInstance().getTargetOnMovePose();
        Logger.recordOutput(super.pb.makePath("currentTargetPose"), currentTargetPose);
        break;

      case TUNING:
        tuningAngle = Degrees.of(SmartDashboard.getNumber("Tuning_Mode/Turret_Tuning_Position_Degrees", 0.0));
        // Implement tuning state behavior here
        CommandScheduler.getInstance().schedule(this.setAngle(() -> tuningAngle));
        break;

      case DUMB_SHOT:
      case STOWED:
        tuningAngle = Degrees.of(0);
        CommandScheduler.getInstance().schedule(this.setAngle(() -> tuningAngle));
        break;
      
      case IDLE:
      default:
        //super.stop(); // Does this not stop Closed loop control???
        break;
    }

    Logger.recordOutput(pb.makePath("goalVector"), new Pose3d[] {this.getGlobalPose(), currentTargetPose});

  };

  @Override
  public Transform3d getTransform3d() {
    
    Angle rotations;
    
    // In simulation, use the target angle instead of encoder position for instant response
    // if (frc319.robot.Robot.isSimulation()) {
    //   rotations = targetAngle;
    //   Logger.recordOutput(pb.makePath("sim_using_target"), true);
    // } else {
      rotations = super.getCurrentPosition().times(config.unitToRotorRatio);
      Logger.recordOutput(pb.makePath("sim_using_target"), false);
    // }
    
    Logger.recordOutput(pb.makePath("motor_rotations"), super.getCurrentPosition().in(Rotations));
    Logger.recordOutput(pb.makePath("turret_rotations"), rotations.in(Rotations));
    Logger.recordOutput(pb.makePath("turret_local_radians"), rotations.in(Radians));
    
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
    //double normalizedRadians = MathUtil.angleModulus(relativeRadians);

    // I want the wrap around to be at a different point to I will clamp it 
    double wrapAroundPoint = (7*Math.PI / 4); // Wrap around at 90 degrees instead of 180
    
    double normalizedRadians = MathUtil.inputModulus(relativeRadians, -2*Math.PI + wrapAroundPoint, wrapAroundPoint);



    return Radians.of(normalizedRadians);
  }

  // In Turret — replaces getCurrentTargetAngleWithVisionCorrection()
public Angle getTurretAngleFromVision() {
    Pose3d camPose = LauncherVisionManager.getInstance().getTurretCameraPoseInField();
    Translation3d camPos = camPose.getTranslation();
    double cameraYawField = camPose.getRotation().getZ();

    Translation3d target = LaunchingSolutionManager.getInstance().getTargetPose().getTranslation();
    double fieldAngleToTarget = Math.atan2(
        target.getY() - camPos.getY(),
        target.getX() - camPos.getX()
    );

    // Delta in camera frame, applied to current encoder position
    double delta = fieldAngleToTarget - cameraYawField;
    double currentTurretAngle = getCurrentPosition().times(config.unitToRotorRatio).in(Radians);
    double newSetpoint = currentTurretAngle + delta;

    double wrapAroundPoint = (7*Math.PI / 4); // Wrap around at 90 degrees instead of 180

    Logger.recordOutput(pb.makePath("vision/fieldAngleToTarget"), Math.toDegrees(fieldAngleToTarget));
    Logger.recordOutput(pb.makePath("vision/cameraYawField"), Math.toDegrees(cameraYawField));
    Logger.recordOutput(pb.makePath("vision/delta"), Math.toDegrees(delta));
    Logger.recordOutput(pb.makePath("vision/newSetpoint"), Math.toDegrees(newSetpoint));

    return Radians.of(MathUtil.inputModulus(newSetpoint, -2*Math.PI + wrapAroundPoint, wrapAroundPoint));
}

public Angle getCurrentTargetAngleWithVisionCorrection() {
    // Fall back to pose-based if vision not ready
    if (!LauncherVisionManager.getInstance().isTurretPoseValid()) {
        return getCurrentTargetAngle();
    }

    Pose3d camPose = LauncherVisionManager.getInstance().getTurretCameraPoseInField();
    Translation3d camPos = camPose.getTranslation();
    double cameraFieldYaw = camPose.getRotation().getZ();

    Translation3d target = LaunchingSolutionManager.getInstance().getTargetPose().getTranslation();
    double fieldAngleToTarget = Math.atan2(
        target.getY() - camPos.getY(),
        target.getX() - camPos.getX()
    );

    double currentTurretAngle = getCurrentPosition().times(config.unitToRotorRatio).in(Radians);
    double newSetpoint = currentTurretAngle + (fieldAngleToTarget - cameraFieldYaw);

    double wrapAroundPoint = (7 * Math.PI / 4);
    double normalizedSetpoint = MathUtil.inputModulus(newSetpoint, -2 * Math.PI + wrapAroundPoint, wrapAroundPoint);

    Logger.recordOutput(pb.makePath("vision/fieldAngleToTarget"), Math.toDegrees(fieldAngleToTarget));
    Logger.recordOutput(pb.makePath("vision/cameraFieldYaw"), Math.toDegrees(cameraFieldYaw));
    Logger.recordOutput(pb.makePath("vision/delta"), Math.toDegrees(fieldAngleToTarget - cameraFieldYaw));
    Logger.recordOutput(pb.makePath("vision/normalizedSetpoint"), Math.toDegrees(normalizedSetpoint));

    return Radians.of(normalizedSetpoint);
}

  public Angle getLauncOnTheFlyAngle() {
    Translation3d diff = this.getTranslationTo(LaunchingSolutionManager.getInstance().getTargetOnMovePose().getTranslation());
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
    //double normalizedRadians = MathUtil.angleModulus(relativeRadians);

    // I want the wrap around to be at a different point to I will clamp it 
    double wrapAroundPoint = (7*Math.PI / 4); // Wrap around at 90 degrees instead of 180
    
    double normalizedRadians = MathUtil.inputModulus(relativeRadians, -2*Math.PI + wrapAroundPoint, wrapAroundPoint);



    return Radians.of(normalizedRadians);
  }

  public boolean isAtTargetPosition() {
    Angle currentAngle = super.getCurrentPosition().times(config.unitToRotorRatio).plus(manualNudgeAngle);
    //Angle targetAngle = Degrees.of(0.0);

    Logger.recordOutput(pb.makePath("iat_currentAngle"), currentAngle.in(Degrees));
    Logger.recordOutput(pb.makePath("iat_targetAngle"), targetAngle.in(Degrees));

    switch (laucherState) {

      case TRACKING_TARGET:
        //targetAngle = getCurrentTargetAngle();

        break;
      
      case TRACK_HUB_ON_MOVE:
       //targetAngle = getLauncOnTheFlyAngle();
       break;

      case DUMB_SHOT:
        targetAngle = Degrees.of(0);

        break;

      default:
        return false; // If we're not actively tracking, we can consider ourselves "at target"
    }

    return EqualsUtil.epsilonEquals(currentAngle.in(Degrees), targetAngle.in(Degrees), 10.0); 
  }

public boolean isAtTargetVelocity() {
    // Get turret's angular velocity relative to the robot chassis
    AngularVelocity turretVelocityLocal = super.getCurrentVelocity().times(config.unitToRotorRatio);
    
    // Get drivetrain's angular velocity (rotation rate) from kinematics manager
    // This is the chassis spinning (ID 0), which affects the turret's global velocity
    Translation3d driveAngularVelocity = KinematicsManager.getInstance().getGlobalAngularVelocity(0);
    double drivetrainAngularVelRadPerSec = driveAngularVelocity.getZ(); // Z-axis is yaw rotation
    
    // Calculate turret's angular velocity in the global/field coordinate system
    // Global velocity = local turret velocity + drivetrain rotation
    double turretGlobalVelocityRadPerSec = turretVelocityLocal.in(RadiansPerSecond) + drivetrainAngularVelRadPerSec;
    
    // Convert to RPM for logging and comparison
    double turretGlobalVelocityRPM = Math.toDegrees(turretGlobalVelocityRadPerSec) * 60.0 / 360.0;
    
    Logger.recordOutput(pb.makePath("iat_turretVelocityLocal"), turretVelocityLocal.in(RPM));
    Logger.recordOutput(pb.makePath("iat_drivetrainAngularVel"), Math.toDegrees(drivetrainAngularVelRadPerSec) * 60.0 / 360.0);
    Logger.recordOutput(pb.makePath("iat_turretVelocityGlobal"), turretGlobalVelocityRPM);
    
    // Check if global velocity is below threshold (e.g., 5 RPM)
    double maxAllowedVelocityRPM = 5.0;
    return Math.abs(turretGlobalVelocityRPM) < maxAllowedVelocityRPM; 
  }

}
