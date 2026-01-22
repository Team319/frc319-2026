// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.launcher;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.Volts;


/** Add your docs here. */
public class LauncherIOSim implements LauncherIO {
    
    private final DCMotor turnMotor = DCMotor.getKrakenX60(1);
    private final SingleJointedArmSim turretSim =
            new SingleJointedArmSim(turnMotor, 2, 0.001, 0.1, 0, 2 * Math.PI, false, 0, 0.0, 0.0);

    private final DCMotor hoodMotor = DCMotor.getKrakenX60(1);
    private final SingleJointedArmSim hoodSim =
            new SingleJointedArmSim(hoodMotor, 2, 0.005, 0.1, 0, Math.PI, false, 0, 0.0002, 0.0002);

    private final DCMotor flywheelMotor = DCMotor.getKrakenX60(1);
    private final FlywheelSim flywheelSim =
            new FlywheelSim(LinearSystemId.createFlywheelSystem(flywheelMotor, 0.005, 2), flywheelMotor, 0.0005);

    private final DCMotor shootMotor = DCMotor.getKrakenX60(1);
    private final FlywheelSim shootSim =
            new FlywheelSim(LinearSystemId.createFlywheelSystem(shootMotor, 0.001, 2), shootMotor, 0.0005);


    public LauncherIOSim()
    {
        setTurnOutput(Volts.zero());
        setHoodOutput(Volts.zero());
        setFlywheelOutput(Volts.zero());
        setShootOutput(Volts.zero());
    }

    @Override
    public void updateInputs(LauncherIOInputs inputs) 
    {
        turretSim.update(0.02);
        hoodSim.update(0.02);
        flywheelSim.update(0.02);
        shootSim.update(0.02);
    }

     @Override
    public void setTurnOutput(Voltage out) {
        turretSim.setInput(out.in(Volts));
    }

    @Override
    public void setHoodOutput(Voltage out) {
        hoodSim.setInput(out.in(Volts));
    }

    @Override
    public void setFlywheelOutput(Voltage out) {
        flywheelSim.setInputVoltage(out.in(Volts));
    }

    @Override
    public void setShootOutput(Voltage out) {
        shootSim.setInputVoltage(out.in(Volts));
    }





}
