package frc319.lib.util;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc319.robot.FieldConstants;

/**
 * Utility class for determining robot position relative to field zones and boundaries.
 * 
 * <p>Field coordinate system (from blue alliance perspective):
 * <ul>
 *   <li>X-axis: Runs lengthwise down the field (0 at blue alliance wall)
 *   <li>Y-axis: Runs widthwise across the field (0 at right wall, increases left)
 * </ul>
 */
public class FieldUtils {

  /** Private constructor to prevent instantiation of utility class */
  private FieldUtils() {
    throw new UnsupportedOperationException("This is a utility class and cannot be instantiated");
  }

  // ==================== Field Side Detection (Y-axis) ====================

  /**
   * Determines if the robot is on the left side of the field (higher Y values).
   * 
   * @param robotPose The current pose of the robot
   * @return true if the robot is on the left half of the field
   */
  public static boolean isLeftSide(Pose2d robotPose) {
    
    if(AllianceUtils.isBlueAlliance()){
      return robotPose.getY() > FieldConstants.LinesHorizontal.center;
    } else {
      return robotPose.getY() <= FieldConstants.LinesHorizontal.center;
    }
  }

  /**
   * Determines if the robot is on the right side of the field (lower Y values).
   * 
   * @param robotPose The current pose of the robot
   * @return true if the robot is on the right half of the field
   */
  public static boolean isRightSide(Pose2d robotPose) {
    if(AllianceUtils.isBlueAlliance()){
      return robotPose.getY() <= FieldConstants.LinesHorizontal.center;
    } else {
      return robotPose.getY() > FieldConstants.LinesHorizontal.center;
    }
  }

  /**
   * Determines if the robot is on the left side of the field (higher Y values).
   * 
   * @param robotPosition The current position of the robot
   * @return true if the robot is on the left half of the field
   */
  public static boolean isLeftSide(Translation2d robotPosition) {
    if(AllianceUtils.isBlueAlliance()){
      return robotPosition.getY() > FieldConstants.LinesHorizontal.center;
    } else {
      return robotPosition.getY() <= FieldConstants.LinesHorizontal.center;
    }
  }

  /**
   * Determines if the robot is on the right side of the field (lower Y values).
   * 
   * @param robotPosition The current position of the robot
   * @return true if the robot is on the right half of the field
   */
  public static boolean isRightSide(Translation2d robotPosition) {
    if(AllianceUtils.isBlueAlliance()){
      return robotPosition.getY() <= FieldConstants.LinesHorizontal.center;
    } else {
      return robotPosition.getY() > FieldConstants.LinesHorizontal.center;
    }
  }

  // ==================== Field Zone Detection (X-axis) ====================

  /**
   * Determines if the robot is closer to the blue alliance side than the red alliance side.
   * 
   * @param robotPose The current pose of the robot
   * @return true if the robot is closer to blue side (X < field center), false if closer to red side
   */
  public static boolean isCloserToBlueSide(Pose2d robotPose) {
    return robotPose.getX() < FieldConstants.LinesVertical.center;
  }

  /**
   * Determines if the robot is closer to the red alliance side than the blue alliance side.
   * 
   * @param robotPose The current pose of the robot
   * @return true if the robot is closer to red side (X >= field center), false if closer to blue side
   */
  public static boolean isCloserToRedSide(Pose2d robotPose) {
    return robotPose.getX() >= FieldConstants.LinesVertical.center;
  }

  /**
   * Determines if the robot is closer to the blue alliance side than the red alliance side.
   * 
   * @param robotPosition The current position of the robot
   * @return true if the robot is closer to blue side (X < field center), false if closer to red side
   */
  public static boolean isCloserToBlueSide(Translation2d robotPosition) {
    return robotPosition.getX() < FieldConstants.LinesVertical.center;
  }

  /**
   * Determines if the robot is closer to the red alliance side than the blue alliance side.
   * 
   * @param robotPosition The current position of the robot
   * @return true if the robot is closer to red side (X >= field center), false if closer to blue side
   */
  public static boolean isCloserToRedSide(Translation2d robotPosition) {
    return robotPosition.getX() >= FieldConstants.LinesVertical.center;
  }

  /**
   * Determines if the robot is in the blue alliance zone (low X values).
   * 
   * @param robotPose The current pose of the robot
   * @return true if the robot is in the blue alliance zone
   */
  public static boolean isInAllianceZone(Pose2d robotPose) {
    if (AllianceUtils.isBlueAlliance()) {
      return robotPose.getX() <= AllianceFlipUtil.applyX(FieldConstants.LinesVertical.allianceZone);
    } else {
      return robotPose.getX() >= AllianceFlipUtil.applyX(FieldConstants.LinesVertical.allianceZone);
    }
  }

  /**
   * Determines if the robot is in the neutral zone (middle of the field).
   * 
   * @param robotPose The current pose of the robot
   * @return true if the robot is in the neutral zone
   */
  public static boolean isInNeutralZone(Pose2d robotPose) {
    double x = robotPose.getX();
    return x > FieldConstants.LinesVertical.neutralZoneNear 
        && x < FieldConstants.LinesVertical.neutralZoneFar;
  }

  /**
   * Determines if the robot is in the opposing alliance zone (high X values).
   * 
   * @param robotPose The current pose of the robot
   * @return true if the robot is in the opposing alliance zone
   */
  public static boolean isInOpposingZone(Pose2d robotPose) {
    if (AllianceUtils.isBlueAlliance()) {
      return robotPose.getX() >= AllianceFlipUtil.applyX(FieldConstants.LinesVertical.oppAllianceZone);
    } else {
      return robotPose.getX() <= AllianceFlipUtil.applyX(FieldConstants.LinesVertical.oppAllianceZone);
    }
  }

  /**
   * Determines if the robot is in the blue alliance zone (low X values).
   * 
   * @param robotPosition The current position of the robot
   * @return true if the robot is in the blue alliance zone
   */
  public static boolean isInAllianceZone(Translation2d robotPosition) {
    if(AllianceUtils.isBlueAlliance()){
      return robotPosition.getX() <= AllianceFlipUtil.applyX(FieldConstants.LinesVertical.allianceZone);
    } else {
      return robotPosition.getX() >= AllianceFlipUtil.applyX(FieldConstants.LinesVertical.allianceZone);
    }
  }

  /**
   * Determines if the robot is in the neutral zone (middle of the field).
   * 
   * @param robotPosition The current position of the robot
   * @return true if the robot is in the neutral zone
   */
  public static boolean isInNeutralZone(Translation2d robotPosition) { 
    double x = robotPosition.getX();
    return x > FieldConstants.LinesVertical.neutralZoneNear 
        && x < FieldConstants.LinesVertical.neutralZoneFar;
  }

  /**
   * Determines if the robot is in the opposing alliance zone (high X values).
   * 
   * @param robotPosition The current position of the robot
   * @return true if the robot is in the opposing alliance zone
   */
  public static boolean isInOpposingZone(Translation2d robotPosition) {
    if(AllianceUtils.isBlueAlliance()){
      return robotPosition.getX() >= AllianceFlipUtil.applyX(FieldConstants.LinesVertical.oppAllianceZone);
    } else {
      return robotPosition.getX() <= AllianceFlipUtil.applyX(FieldConstants.LinesVertical.oppAllianceZone);
    }
  }

  // ==================== Combined Zone Detection ====================

  /**
   * Gets a descriptive string of the robot's current field zone.
   * 
   * @param robotPose The current pose of the robot
   * @return A string describing the zone (e.g., "Left Alliance Zone", "Right Neutral Zone")
   */
  public static String getZoneDescription(Pose2d robotPose) {
    String side = isLeftSide(robotPose) ? "Left" : "Right";
    String zone;
    
    if (isInAllianceZone(robotPose)) {
      zone = "Alliance Zone";
    } else if (isInNeutralZone(robotPose)) {
      zone = "Neutral Zone";
    } else if (isInOpposingZone(robotPose)) {
      zone = "Opposing Zone";
    } else {
      zone = "Unknown Zone";
    }
    
    return side + " " + zone;
  }


// ==================== Specific Area Detection ====================
  /**
   * Determines if the robot is in the left trench opening area.
   * 
   * @param robotPose The current pose of the robot
   * @return true if the robot is in the left trench opening area
   */
  public static boolean isInLeftTrenchOpening(Translation2d robotPose) {
    double x = robotPose.getX();
    double y = robotPose.getY();
    if(AllianceUtils.isBlueAlliance()){
      return x >= AllianceFlipUtil.applyX(FieldConstants.LinesVertical.hubCenter) - 1.0 
          && x <= AllianceFlipUtil.applyX(FieldConstants.LinesVertical.hubCenter) + 1.0
          && y >= AllianceFlipUtil.applyY(FieldConstants.LinesHorizontal.leftTrenchOpenEnd) - 1.0
          && y <= AllianceFlipUtil.applyY(FieldConstants.LinesHorizontal.leftTrenchOpenStart);
    } else {
    return x >= AllianceFlipUtil.applyX(FieldConstants.LinesVertical.hubCenter) - 1.0 
        && x <= AllianceFlipUtil.applyX(FieldConstants.LinesVertical.hubCenter) + 1.0
        && y >= AllianceFlipUtil.applyY(FieldConstants.LinesHorizontal.leftTrenchOpenEnd) - 1.0
        && y <= AllianceFlipUtil.applyY(FieldConstants.LinesHorizontal.leftTrenchOpenStart);
    }
  }

  /**
   * Determines if the robot is in the right trench opening area.
   * 
   * @param robotPose The current pose of the robot
   * @return true if the robot is in the right trench opening area
   */
  public static boolean isInRightTrenchOpening(Translation2d robotPose) {
    double x = robotPose.getX();
    double y = robotPose.getY();
    if(AllianceUtils.isBlueAlliance()){
      return x >= AllianceFlipUtil.applyX(FieldConstants.LinesVertical.hubCenter) - 1.0 
          && x <= AllianceFlipUtil.applyX(FieldConstants.LinesVertical.hubCenter) + 1.0
          && y >= AllianceFlipUtil.applyY(FieldConstants.LinesHorizontal.rightTrenchOpenEnd)
          && y <= AllianceFlipUtil.applyY(FieldConstants.LinesHorizontal.rightTrenchOpenStart) + 1.0;
    } else {
    return x >= AllianceFlipUtil.applyX(FieldConstants.LinesVertical.hubCenter) - 1.0 
        && x <= AllianceFlipUtil.applyX(FieldConstants.LinesVertical.hubCenter) + 1.0
        && y >= AllianceFlipUtil.applyY(FieldConstants.LinesHorizontal.rightTrenchOpenEnd)
        && y <= AllianceFlipUtil.applyY(FieldConstants.LinesHorizontal.rightTrenchOpenStart) + 1.0;
    }
  }

  /**
   * Determines if the robot is in ANY trench opening area (left, right, alliance side, or opposing side).
   * 
   * @param robotPose The current pose of the robot
   * @return true if the robot is in any trench opening area
   */
  public static boolean isInAnyTrenchOpening(Translation2d robotPose) {
    return isInLeftTrenchOpening(robotPose) || isInRightTrenchOpening(robotPose);
  }

}
