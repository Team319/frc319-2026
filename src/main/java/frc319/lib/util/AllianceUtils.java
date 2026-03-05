package frc319.lib.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;

/**
 * Utility class for retrieving alliance information from the Driver Station.
 */
public class AllianceUtils {

  /** Private constructor to prevent instantiation of utility class */
  private AllianceUtils() {
    throw new UnsupportedOperationException("This is a utility class and cannot be instantiated");
  }

  /**
   * Gets the current alliance color from the Driver Station.
   * 
   * @return An Optional containing the Alliance (RED or BLUE) if present, or empty if not available
   */
  public static Optional<Alliance> getAlliance() {
    return DriverStation.getAlliance();
  }

  /**
   * Checks if the current alliance is Blue.
   * 
   * @return true if the alliance is Blue, false otherwise (including when alliance is not available)
   */
  public static boolean isBlueAlliance() {
    return getAlliance().map(alliance -> alliance == Alliance.Blue).orElse(false);
  }

  /**
   * Checks if the current alliance is Red.
   * 
   * @return true if the alliance is Red, false otherwise (including when alliance is not available)
   */
  public static boolean isRedAlliance() {
    return getAlliance().map(alliance -> alliance == Alliance.Red).orElse(false);
  }
}
