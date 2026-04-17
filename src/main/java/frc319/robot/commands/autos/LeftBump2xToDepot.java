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
public class LeftBump2xToDepot {

  public static Command getCommand(Drive drive) {

   double defaultHeading = 270.0;

   double waitTimeInNeutralZone = 0.5;
    
   return 
    Commands.sequence(
      Commands.runOnce(() -> System.out.println("Running Left Bump to Depot Auto")),
      Commands.runOnce(() -> drive.setPose(AllianceFlipUtil.apply(new Pose2d(new Translation2d(3.5, 5.6), new Rotation2d(Math.toRadians(defaultHeading)))))),
      Commands.waitSeconds(0.5),
      Commands.parallel(
        DriveCommands.followPathCommand("go_left_bump2_part1",false),
        Commands.waitSeconds(2.0)
        .andThen(Commands.runOnce(()-> Robot.m_robotContainer.setRobotState(RobotStates.COLLECTING)))
      ),// end of parallel
      Commands.waitSeconds(waitTimeInNeutralZone),
      DriveCommands.followPathCommand("go_left_bump2_part2",false)
      .andThen(Commands.runOnce(()-> Robot.m_robotContainer.setRobotState(RobotStates.SHOOTING_ON_MOVE)))

    ); // end of sequence
  }
}
        // .andThen(Commands.waitSeconds(10.0))
