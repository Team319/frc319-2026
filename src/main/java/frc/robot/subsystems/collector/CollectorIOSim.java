// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.collector;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/** Add your docs here. */
public class CollectorIOSim implements CollectorIO {

        private final DCMotor pivotMotor = DCMotor.getKrakenX60(1);
    private final SingleJointedArmSim pivotSim =
            new SingleJointedArmSim(pivotMotor, 2, 0.005, 0.1, 0, Math.PI, false, 0, 0.0002, 0.0002);

    private final DCMotor rollerMotor = DCMotor.getKrakenX60(1);
    private final FlywheelSim rollerSim =
            new FlywheelSim(LinearSystemId.createFlywheelSystem(rollerMotor, 0.005, 2), rollerMotor, 0.0005);

    @Override
    public void updateInputs(CollectorIOInputs inputs) 
    {
        pivotSim.update(0.02);
        rollerSim.update(0.02);
    }

    @Override
    public void setRollerOutput(Voltage output) {

    }

    @Override
    public void setPivotOutput(Voltage output) {

    }

    // Stop all collector motion
    @Override
    public void stop () {

    }

    // Stop the roller motion
    @Override
    public void stopRoller() {

    }

    // Stop the pivot motion
    @Override
    public void stopPivot() {

    }

    // Collect game pieces
    @Override
    public void collect() {

    }

    // Eject game pieces
    @Override
    public void eject() {

    }

    // Extend the collector mechanism
    @Override
    public void extend () {

    }

    // Retract the collector mechanism
    @Override
    public void retract () {

    }

}
