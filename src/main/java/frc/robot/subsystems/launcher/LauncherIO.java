// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.launcher;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.units.measure.Voltage;


/** Add your docs here. */
public interface LauncherIO {

  @AutoLog
  public static class LauncherIOInputs 
  {
    public double turretAngleRadians = 0.0;
    public double hoodAngleRadians = 0.0;
    public double flywheelVelocityRpm = 0.0;
    public double kickerVelocityRpm = 0.0;
  }

  // Update the set of inputs for AdvantageKit logging
  public default void updateInputs( LauncherIOInputs inputs ) {}
  
  public default void setTurnOutput( Voltage out ) {};

  public default void setHoodOutput( Voltage out ) {};

  public default void setFlywheelOutput( Voltage out ) {};

  public default void setKickerOutput( Voltage out ) {};

  public default void stopTurn() {}

  public default void stopHood() {}

  public default void stopFlywheel() {}

  public default void stopShoot() {}

  public default void resetTurnEncoder() {}

}
