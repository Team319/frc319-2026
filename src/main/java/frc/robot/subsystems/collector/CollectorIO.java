// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.collector;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Voltage;


/** Add your docs here. */
public interface CollectorIO {

  @AutoLog
  public static class CollectorIOInputs 
  {
    //public boolean extended = false;
  }

  // Update the set of inputs for AdvantageKit logging
  public default void updateInputs(CollectorIOInputs inputs) {}

  public default void setRollerOutput(Voltage output) {}

  public default void setPivotOutput(Voltage output) {}

  // Stop all collector motion
  public default void stop () {}

  // Stop the roller motion
  public default void stopRoller() {}

  // Stop the pivot motion
  public default void stopPivot() {}

  // Collect game pieces
  public default void collect() {}

  // Eject game pieces
  public default void eject() {}

  // Extend the collector mechanism
  public default void extend () {}

  // Retract the collector mechanism
  public default void retract () {}





}
