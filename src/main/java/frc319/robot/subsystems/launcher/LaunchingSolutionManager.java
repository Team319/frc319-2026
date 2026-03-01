package frc319.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.lang.annotation.Target;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc319.redhawk_lib.subsystem.KinematicsManager;
import frc319.robot.FieldConstants;
import frc319.robot.util.AllianceFlipUtil;
import frc319.robot.util.FieldUtils;

public class LaunchingSolutionManager extends SubsystemBase {
  private static LaunchingSolutionManager instance;

  // Target Pose (default to blue origin)
  private Pose3d targetPose = new Pose3d();
  private Pose3d targetOnMovePose = new Pose3d();

  private class TargetPoses {
    public static final Pose3d hubTopCenter =
        new Pose3d(FieldConstants.Hub.topCenterPoint, new Rotation3d());

    public static final Pose3d allianceLeft =
        new Pose3d(new Translation3d(FieldConstants.LinesVertical.allianceZone/2,3*(FieldConstants.LinesHorizontal.center/2),1), new Rotation3d());
    public static final Pose3d allianceRight =
        new Pose3d(new Translation3d(FieldConstants.LinesVertical.allianceZone/2,(FieldConstants.LinesHorizontal.center/2),1), new Rotation3d());

  }

  // --- Data Structures ---
  public static record LaunchSolution(
      Rotation2d turretFieldRelativeYaw, // Target global yaw for the turret
      double flywheelSpeedMetersPerSecond, // Required exit velocity
      Rotation2d hoodPitch, // Required vertical angle
      double effectiveDistanceMeters,
      boolean isValid // False if target is out of range or blocked
      ) {}

  // Default to an empty/invalid solution
  private LaunchSolution currentSolution =
      new LaunchSolution(new Rotation2d(), 0, new Rotation2d(), 0, true); // set me to false

  public LaunchingSolutionManager() {
    if (instance != null) {
      throw new IllegalStateException("LaunchingSolutionManager already initialized!");
    }
    instance = this;
  }

  public static LaunchingSolutionManager getInstance() {
    // Lazy load logic could go here, but for Subsystems, constructor is better
    return instance;
  }

public void calculateCurrentTargets(){
// 1. Get Robot State (ID 0 = Chassis)
    Pose3d robotPose = KinematicsManager.getInstance().getGlobalPose(0);
    Translation3d robotVel = KinematicsManager.getInstance().getGlobalLinearVelocity(0);

    // 2. Solve for the Launch Vector
        // Pick a starting goal based on alliance color

    if( FieldUtils.isInAllianceZone(robotPose.toPose2d() )){
      targetPose = AllianceFlipUtil.apply(new Pose3d(FieldConstants.Hub.topCenterPoint, new Rotation3d()));
    }
    else{
      if(FieldUtils.isLeftSide(robotPose.toPose2d())){
        targetPose = AllianceFlipUtil.apply((TargetPoses.allianceLeft));
      }
      else // mens we're on the right side of the field
      {
         targetPose = AllianceFlipUtil.apply(TargetPoses.allianceRight);
      }
    }
    
    Logger.recordOutput("LaunchingSolutionManager/currentTargetPose", new Pose3d[] {this.targetPose});

    currentSolution = calculate(robotPose, robotVel, targetPose.getTranslation());

    targetOnMovePose = new Pose3d(this.targetPose.getTranslation().minus(robotVel), new Rotation3d());
    Logger.recordOutput("LaunchingSolutionManager/currentSolution/onMoveTargetPose", targetOnMovePose);
  }

    //==============================================================
  @Override
  public void periodic() {
    calculateCurrentTargets();

  }

  public LaunchSolution getSolution() {
    return currentSolution;
  }

  private LaunchSolution calculate(
      Pose3d robotPose, Translation3d robotVel, Translation3d targetPos) {
    // A. Relative Position
    Translation3d rangeVec = targetPos.minus(robotPose.getTranslation());
    double dist = rangeVec.getNorm();

    // B. Check Range
    if (dist > 8.0 || dist < 1.0) {
      return new LaunchSolution(new Rotation2d(), 0, new Rotation2d(), dist, true); // TODO - this was set false
    }

    // C. Get Ideal Static Launch Params (Ground Relative)
    double idealSpeed =
        FeetPerSecond.of(LauncherConstants.Flywheels.velocityMap.get(dist)).in(MetersPerSecond);
    double idealPitchRad = Math.toRadians(LauncherConstants.Hood.angleMap.get(dist));

    // D. Construct Ideal Velocity Vector
    // Normalized horizontal direction to goal
    Translation3d horizontalDir =
        new Translation3d(rangeVec.getX(), rangeVec.getY(), 0)
            .div(rangeVec.toTranslation2d().getNorm());

    // Combine Horizontal and Vertical components
    Translation3d idealVelocity =
        horizontalDir
            .times(idealSpeed * Math.cos(idealPitchRad))
            .plus(new Translation3d(0, 0, idealSpeed * Math.sin(idealPitchRad)));

    // E. Subtract Robot Velocity (V_muzzle = V_ideal - V_robot)
    Translation3d neededMuzzleVelocity = idealVelocity.minus(robotVel);

    // F. Extract Parameters from Resulting Vector
    double newSpeed = neededMuzzleVelocity.getNorm();

    // Vertical Angle (Pitch)
    double newPitch =
        Math.atan2(
            neededMuzzleVelocity.getZ(),
            Math.hypot(neededMuzzleVelocity.getX(), neededMuzzleVelocity.getY()));

    // Horizontal Angle (Yaw)
    double newYaw = Math.atan2(neededMuzzleVelocity.getY(), neededMuzzleVelocity.getX());

    return new LaunchSolution(
        new Rotation2d(newYaw), newSpeed, new Rotation2d(newPitch), dist, true);
  }

  public Pose3d getTargetPose() {
    return targetPose;
  }

  public Pose3d getTargetOnMovePose() {
    return targetOnMovePose;
  }
}
