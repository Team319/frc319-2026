package frc319.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc319.lib.io.TalonFXIO;
import frc319.lib.subsystem.MotorFollowerSubsystem;
import frc319.lib.subsystem.TalonFXSubsystemConfig;
import frc319.lib.io.MotorInputsAutoLogged;
import frc319.robot.subsystems.intake.IntakeConstants.IntakeStates;

public class IntakeRollers extends MotorFollowerSubsystem<MotorInputsAutoLogged, TalonFXIO> {

  IntakeStates intakeState = IntakeStates.IDLE;
  private double currentDutyCycle = 0.0;

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

  public void setState(IntakeStates state){
    this.intakeState = state;
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
    Logger.recordOutput(pb.makePath("state"), intakeState);


    super.periodic();
    switch(intakeState){
      case COLLECT:
        this.intake().schedule();
        break;

      case EJECT:
        this.outtake().schedule();
        break;
      
      case RETRACTED:
      case EXTENDED:
        // For now... no change ?
        // probably want to check if ejecting and stop or something...
        // may need to keep track of last state. 
        break;


      case TUNING:
         // For now... no change ?
        // probably want to check if ejecting and stop or something...
        // may need to keep track of last state. 
        currentDutyCycle = SmartDashboard.getNumber("Tuning_Mode/IntakeRollers_tuning_DutyCycle", 0.0);
        this.setIntakeSpeed(currentDutyCycle).schedule();
        break;

      case IDLE:
      default:
        this.stop().schedule();
        break;
    }
    
  }
}
