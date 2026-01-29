// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

/* import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.GenericEntry;*/
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
/*import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;*/
//import frc.robot.Constants;
import frc.robot.Constants.LimelightConstants;

public class Limelight {

  private static final NetworkTable m_turretTable = NetworkTableInstance.getDefault().getTable("limelight-turret"); // Makes the Limelight data table.
  private static NetworkTableEntry m_botPose_turret = m_turretTable.getEntry("botpose_wpiblue"); // Makes a double array to hold the robot pose.
  private static NetworkTableEntry m_tv_turret = m_turretTable.getEntry("tv");
  private static NetworkTableEntry m_tx_turret = m_turretTable.getEntry("tx");
  private static NetworkTableEntry m_ty_turret = m_turretTable.getEntry("ty");
  private static NetworkTableEntry m_ta_turret = m_turretTable.getEntry("ta");
  private static NetworkTableEntry m_tl_turret = m_turretTable.getEntry("tl");
  private static NetworkTableEntry m_cl_turret = m_turretTable.getEntry("cl");

  private static final NetworkTable m_drivetrainTable = NetworkTableInstance.getDefault().getTable("limelight-drivetrain"); // Makes the Limelight data table.
  private static NetworkTableEntry m_botPose_drivetrain = m_drivetrainTable.getEntry("botpose_wpiblue"); // Makes a double array to hold the robot pose.
  private static NetworkTableEntry m_tv_drivetrain = m_drivetrainTable.getEntry("tv");
  private static NetworkTableEntry m_tx_drivetrain = m_drivetrainTable.getEntry("tx");
  private static NetworkTableEntry m_ty_drivetrain = m_drivetrainTable.getEntry("ty");
  private static NetworkTableEntry m_ta_drivetrain = m_drivetrainTable.getEntry("ta");
  private static NetworkTableEntry m_tl_drivetrain = m_drivetrainTable.getEntry("tl");
  private static NetworkTableEntry m_cl_drivetrain = m_drivetrainTable.getEntry("cl");

 // private NetworkTableEntry m_

  /** Creates a new Limelight. */
  public Limelight() {}

  //Returns latency
  public static double getLatency(LimelightConstants.Device device) {
    switch(device){
      case TURRET:
        return m_tl_turret.getDouble(0.0); 
      case DRIVETRAIN:
      default:
        return m_tl_drivetrain.getDouble(0.0);

    }
  }

  //Returns total latency
  public static double getTotalLatency(LimelightConstants.Device device) {
    switch(device){
      case TURRET:
        return m_cl_turret.getDouble(0.0); 
      case DRIVETRAIN:
      default:
        return m_cl_drivetrain.getDouble(0.0);
    }
   }

  //Returns target area
   public static double getTargetArea(LimelightConstants.Device device){
    switch(device){
      case TURRET:
        return m_ta_turret.getDouble(0.0); 
      case DRIVETRAIN:
      default:
        return m_ta_drivetrain.getDouble(0.0); 
    }
    
  }

  // Returns the horizontal offset from valid target
  public static double getHorizontalOffset(LimelightConstants.Device device) {
    switch(device){
      case TURRET:
        return m_tx_turret.getDouble(0.0); 
      case DRIVETRAIN:
      default:
        return m_tx_drivetrain.getDouble(0.0); 
    }
  }

  // Returns the vertical offset from valid target
  public static double getVerticalOffset(LimelightConstants.Device device) {
    switch(device){
      case TURRET:
        return m_ty_turret.getDouble(0.0); 
      case DRIVETRAIN:
      default:
        return m_ty_drivetrain.getDouble(0.0); 
    }
  }

  // Returns the distance from the target
  public static double getDistance(LimelightConstants.Device device) {
    switch(device){
      case TURRET:
      return 0.0;
        //return (Constants.kTargetHeight - Constants.kCameraHeight) / Math.tan(Math.toRadians(Constants.kCameraAngle + getVerticalOffset(LimelightConstants.Device.TURRET))); 
      case DRIVETRAIN:
      default:
      return 0.0;
        //return (Constants.kTargetHeight - Constants.kCameraHeight) / Math.tan(Math.toRadians(Constants.kCameraAngle + getVerticalOffset(LimelightConstants.Device.DRIVETRAIN))); 
    }
  }

  // Returns True if a valid target is seen
  public static boolean isValidTargetSeen (LimelightConstants.Device device) {
    double result = 0.0;
    switch(device){
      case TURRET:
        result = m_tv_turret.getDouble(0.0); 
        break;
      case DRIVETRAIN:
      default:
        result = m_tv_drivetrain.getDouble(0.0); 
        break;
    }

    return result == 1.0;
  }

  //Returns field space robot pose
  public static double[] getBotPose(LimelightConstants.Device device) {
    switch(device){
      case TURRET:
        if (m_botPose_turret.getDoubleArray(new double[7]).length >= 7){
          return m_botPose_turret.getDoubleArray(new double[7]);
        }
        return  m_botPose_turret.getDoubleArray(new double[7]); 
      case DRIVETRAIN:
      default:
        if (m_botPose_drivetrain.getDoubleArray(new double[7]).length >= 7){
          return m_botPose_drivetrain.getDoubleArray(new double[7]);
        }
    }
    // Fallback case
    return new double[7];

  }

}







