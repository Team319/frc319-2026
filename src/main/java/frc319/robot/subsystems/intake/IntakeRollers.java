package frc319.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc319.redhawk_lib.io.MotorInputsAutoLogged;
import frc319.redhawk_lib.io.TalonFXIO;
import frc319.redhawk_lib.subsystem.MotorFollowerSubsystem;
import frc319.redhawk_lib.subsystem.TalonFXSubsystemConfig;

public class IntakeRollers extends MotorFollowerSubsystem<MotorInputsAutoLogged, TalonFXIO> {

  public IntakeRollers(
    final TalonFXSubsystemConfig leadConfig, 
    final TalonFXSubsystemConfig followConfig, 
    final TalonFXIO intakeRollersLeadMotorIO,
    final TalonFXIO intakeRollersFollowMotorIO) {
    super(
      "IntakeRollers",
      leadConfig, 
      followConfig, 
      new MotorInputsAutoLogged(),
      new MotorInputsAutoLogged(), 
      intakeRollersLeadMotorIO, 
      intakeRollersFollowMotorIO
      );
  }

  public Command setIntakeSpeed(double speed) {
    return dutyCycleCommand(() -> speed);
  }

  public Command intake() {
    return setIntakeSpeed(IntakeConstants.Rollers.intakeDutyCycle);
  }

  public Command stop() {
    return setIntakeSpeed(0);
  }

  public Command outtake() {
    return setIntakeSpeed(IntakeConstants.Rollers.outtakeDutyCycle);
  }

  @Override
  public void periodic() {
    super.periodic();
    // Additional periodic code for intake rollers can be added here
  }
}
