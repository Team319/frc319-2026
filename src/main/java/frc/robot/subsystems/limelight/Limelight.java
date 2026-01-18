// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.limelight;

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

  private static final NetworkTable m_reefTable = NetworkTableInstance.getDefault().getTable("limelight-reef"); // Makes the Limelight data table.
  private static NetworkTableEntry m_botPose_reef = m_reefTable.getEntry("botpose_wpiblue"); // Makes a double array to hold the robot pose.
  private static NetworkTableEntry m_tv_reef = m_reefTable.getEntry("tv");
  private static NetworkTableEntry m_tx_reef = m_reefTable.getEntry("tx");
  private static NetworkTableEntry m_ty_reef = m_reefTable.getEntry("ty");
  private static NetworkTableEntry m_ta_reef = m_reefTable.getEntry("ta");
  private static NetworkTableEntry m_tl_reef = m_reefTable.getEntry("tl");
  private static NetworkTableEntry m_cl_reef = m_reefTable.getEntry("cl");

  private static final NetworkTable m_coralTable = NetworkTableInstance.getDefault().getTable("limelight-coral"); // Makes the Limelight data table.
  private static NetworkTableEntry m_botPose_coral = m_coralTable.getEntry("botpose_wpiblue"); // Makes a double array to hold the robot pose.
  private static NetworkTableEntry m_tv_coral = m_coralTable.getEntry("tv");
  private static NetworkTableEntry m_tx_coral = m_coralTable.getEntry("tx");
  private static NetworkTableEntry m_ty_coral = m_coralTable.getEntry("ty");
  private static NetworkTableEntry m_ta_coral = m_coralTable.getEntry("ta");
  private static NetworkTableEntry m_tl_coral = m_coralTable.getEntry("tl");
  private static NetworkTableEntry m_cl_coral = m_coralTable.getEntry("cl");

 // private NetworkTableEntry m_

  /** Creates a new Limelight. */
  public Limelight() {}

  public static double getLatency(LimelightConstants.Device device) {
    switch(device){
      case REEF:
        return m_tl_reef.getDouble(0.0); //Returns latency
      case CORAL_STATION:
      default:
        return m_tl_coral.getDouble(0.0); //Returns latency

    }
  }

  public static double getTotalLatency(LimelightConstants.Device device) {
    switch(device){
      case REEF:
        return m_cl_reef.getDouble(0.0); //Returns total latency
      case CORAL_STATION:
      default:
        return m_cl_coral.getDouble(0.0); //Returns total latency
    }
   }

   public static double getTargetArea(LimelightConstants.Device device){
    switch(device){
      case REEF:
        return m_ta_reef.getDouble(0.0); //Returns target area
      case CORAL_STATION:
      default:
        return m_ta_coral.getDouble(0.0); //Returns target area
    }
    
  }

  public static double getHorizontalOffset(LimelightConstants.Device device) {
    switch(device){
      case REEF:
        return m_tx_reef.getDouble(0.0); // Returns the horizontal offset from valid target
      case CORAL_STATION:
      default:
        return m_tx_coral.getDouble(0.0); // Returns the horizontal offset from valid target
    }
  }

  public static double getVerticalOffset(LimelightConstants.Device device) {
    switch(device){
      case REEF:
        return m_ty_reef.getDouble(0.0); // Returns the vertical offset from valid target
      case CORAL_STATION:
      default:
        return m_ty_coral.getDouble(0.0); // Returns the vertical offset from valid target
    }
  }

  public static double getDistance(LimelightConstants.Device device) {
    switch(device){
      case REEF:
      return 0.0;
        //return (Constants.kTargetHeight - Constants.kCameraHeight) / Math.tan(Math.toRadians(Constants.kCameraAngle + getVerticalOffset(LimelightConstants.Device.REEF))); // Returns the distance from the target
      case CORAL_STATION:
      default:
      return 0.0;
        //return (Constants.kTargetHeight - Constants.kCameraHeight) / Math.tan(Math.toRadians(Constants.kCameraAngle + getVerticalOffset(LimelightConstants.Device.CORAL_STATION))); // Returns the distance from the target
    }
  }

  public static boolean isValidTargetSeen (LimelightConstants.Device device) {
    double result = 0.0;
    switch(device){
      case REEF:
        result = m_tv_reef.getDouble(0.0); // Returns 1 if a valid target is seen
        break;
      case CORAL_STATION:
      default:
        result = m_tv_coral.getDouble(0.0); // Returns 1 if a valid target is seen
        break;
    }

    return result == 1.0;
  }

  public static double[] getBotPose(LimelightConstants.Device device) {
    switch(device){
      case REEF:
        if (m_botPose_reef.getDoubleArray(new double[7]).length >= 7){
          return m_botPose_reef.getDoubleArray(new double[7]);
        }
        return  m_botPose_reef.getDoubleArray(new double[7]); //Returns field space robot pose
      case CORAL_STATION:
      default:
        if (m_botPose_coral.getDoubleArray(new double[7]).length >= 7){
          return m_botPose_coral.getDoubleArray(new double[7]);
        }
    }
    // Fallback case
    return new double[7];

  }

}







