package frc319.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc319.redhawk_lib.io.MotorInputsAutoLogged;
import frc319.redhawk_lib.io.TalonFXIO;
import frc319.redhawk_lib.subsystem.MotorSubsystem;
import frc319.redhawk_lib.subsystem.TalonFXSubsystemConfig;

public class IntakeRoller extends MotorSubsystem<MotorInputsAutoLogged, TalonFXIO> {

  public IntakeRoller(final TalonFXSubsystemConfig config, final TalonFXIO intakeRollersMotorIO) {
    super(config, new MotorInputsAutoLogged(), intakeRollersMotorIO);
  }

  public Command setIntakeSpeed(double speed) {
    return dutyCycleCommand(() -> speed);
  }

  public Command intake() {
    return setIntakeSpeed(IntakeConstants.Roller.intakeDutyCycle);
  }

  public Command stop() {
    return setIntakeSpeed(0);
  }

  public Command outtake() {
    return setIntakeSpeed(IntakeConstants.Roller.outtakeDutyCycle);
  }

  @Override
  public void periodic() {
    super.periodic();
    // Additional periodic code for intake rollers can be added here
  }
}
