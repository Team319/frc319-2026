// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.launcher;

import frc.robot.Constants.LauncherConstants.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.Volts;


/** Add your docs here. */
public class LauncherIOTalonFx implements LauncherIO {
    
    private final TalonFX turnMotor;
    private final TalonFX hoodMotor;
    private final TalonFX flywheelMotor;
    private final TalonFX kickerMotor;

    private final StatusSignal<Angle> turnPosition;
    private final StatusSignal<AngularVelocity> turnVelocity;
    private final StatusSignal<Current> turnCurrent;

    private final StatusSignal<Angle> hoodPosition;
    private final StatusSignal<AngularVelocity> hoodVelocity;
    private final StatusSignal<Current> hoodCurrent;

    private final StatusSignal<AngularVelocity> flywheelSpeed;
    private final StatusSignal<Current> flywheelCurrent;

    private final StatusSignal<AngularVelocity> kickerSpeed;
    private final StatusSignal<Current> kickerCurrent;

    private final VoltageOut turnVoltageRequest = new VoltageOut(0);
    private final VoltageOut hoodVoltageRequest = new VoltageOut(0);
    private final VoltageOut flywheelVoltageRequest = new VoltageOut(0);
    private final VoltageOut kickerVoltageRequest = new VoltageOut(0);

    private final CANBus rioCANBus = new CANBus("rio");



    public LauncherIOTalonFx()
    {

        turnMotor = new TalonFX(21, rioCANBus);
        hoodMotor = new TalonFX(22, rioCANBus);
        flywheelMotor = new TalonFX(23, rioCANBus);
        kickerMotor = new TalonFX(24, rioCANBus);

        turnPosition = turnMotor.getPosition();
        turnVelocity = turnMotor.getVelocity();
        turnCurrent = turnMotor.getStatorCurrent();

        hoodPosition = hoodMotor.getPosition();
        hoodVelocity = hoodMotor.getVelocity();
        hoodCurrent = hoodMotor.getStatorCurrent();

        flywheelSpeed = flywheelMotor.getVelocity();
        flywheelCurrent = flywheelMotor.getStatorCurrent();

        kickerSpeed = kickerMotor.getVelocity();
        kickerCurrent = kickerMotor.getStatorCurrent();
    }

    @Override
    public void updateInputs(LauncherIOInputs inputs) 
    {

    }

     @Override
    public void setTurnOutput(Voltage out) {

    }

    @Override
    public void setHoodOutput(Voltage out) {

    }

    @Override
    public void setFlywheelOutput(Voltage out) {

    }

    @Override
    public void setKickerOutput(Voltage out) {

    }

    @Override
    public void stopTurn() {}

    @Override
    public void stopHood() {}

    @Override
    public void stopFlywheel() {}

    @Override
    public void stopShoot() {}

    @Override
    public void resetTurnEncoder() {}





} // End of LauncherIOTalonFx
