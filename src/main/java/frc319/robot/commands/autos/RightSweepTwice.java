// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc319.robot.commands.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc319.lib.util.AllianceFlipUtil;
import frc319.lib.util.AllianceUtils;
import frc319.robot.FieldConstants;
import frc319.robot.Robot;
import frc319.robot.RobotContainer.RobotStates;
import frc319.robot.commands.DriveCommands;
import frc319.robot.subsystems.drive.Drive;
import frc319.robot.subsystems.intake.IntakeConstants.IntakeStates;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RightSweepTwice {

  static double defaultHeading = 90.0;
  static Pose2d startingPose = AllianceFlipUtil.apply(new Pose2d(new Translation2d(FieldConstants.LinesVertical.starting, 0.575), new Rotation2d(Math.toRadians(defaultHeading))));

  public static Command getCommand(Drive drive) {
    return 
    Commands.sequence(
      Commands.runOnce(() -> System.out.println("Running Right Sweep to Outpost Auto")),
      Commands.runOnce(() -> drive.setPose(startingPose)),
      Commands.parallel(
        DriveCommands.followPathCommand("go_left",true),
        Commands.waitSeconds(1.0)
        .andThen(
          Commands.runOnce(()-> Robot.m_robotContainer.setRobotState(RobotStates.COLLECTING))
        ) 
      ),
      Commands.runOnce(()-> Robot.m_robotContainer.setRobotState(RobotStates.SHOOTING_ON_MOVE)),
      Commands.waitSeconds(0.5),
      Commands.runOnce(()-> Robot.m_robotContainer.intakeExtension.setState(IntakeStates.JOSTLE)),
      Commands.waitSeconds(1.0),
      Commands.runOnce(()-> Robot.m_robotContainer.intakeExtension.setState(IntakeStates.RETRACTED)),
      Commands.waitSeconds(3.0),
      Commands.runOnce(()-> Robot.m_robotContainer.setRobotState(RobotStates.STOP_SHOOTING)),
      Commands.runOnce(()-> Robot.m_robotContainer.setRobotState(RobotStates.COLLECTING)),
      
      DriveCommands.followPathCommand("go_left_mid",true),
      Commands.runOnce(()-> Robot.m_robotContainer.setRobotState(RobotStates.SHOOTING_ON_MOVE)),
      Commands.waitSeconds(0.5),
      Commands.runOnce(()-> Robot.m_robotContainer.intakeExtension.setState(IntakeStates.JOSTLE)),
      Commands.waitSeconds(1.0),
      Commands.runOnce(()-> Robot.m_robotContainer.intakeExtension.setState(IntakeStates.RETRACTED)),
      Commands.waitSeconds(1.5),
      Commands.runOnce(()-> Robot.m_robotContainer.setRobotState(RobotStates.STOP_SHOOTING))
    );
  }
}
