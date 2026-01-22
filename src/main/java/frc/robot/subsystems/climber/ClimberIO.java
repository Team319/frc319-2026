// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ClimberIO {

  @AutoLog
  public static class ClimberIOInputs 
  {
    //public boolean extended = false;
  }

  // Update the set of inputs for AdvantageKit logging
  public default void updateInputs(ClimberIOInputs inputs) {}

  // Stop all collector motion
  public default void stop () {}

  // Extend the collector mechanism
  public default void extend () {}

  // Retract the collector mechanism
  public default void retract () {}

}
