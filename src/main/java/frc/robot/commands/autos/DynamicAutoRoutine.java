// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;


/** Add your docs here. */
public class DynamicAutoRoutine extends SequentialCommandGroup {

    String m_instruction = "";
    Drive m_drive;

public DynamicAutoRoutine(Drive a_drive){
    // Constructor
    m_drive = a_drive;
    //Populate some string from Dashboard with format <ReefPosition><ReefLevel>..." ( ie - A1B2C3D4 ; A4B4C4D4 ; etc )
    m_instruction = SmartDashboard.getString("DynamicAutoInput", "");

    if (m_instruction == null || m_instruction.isEmpty()) {

        System.out.println("request is empty or unexpected... instruction = " + m_instruction);
        m_instruction = "";
    }

    List<Pair<String, Integer>> parsedInstructions = parseInstruction(m_instruction);
    // TODO : Break down instruction

    // Reset robot pose to starting position.



    for (Pair<String, Integer> pair : parsedInstructions) {
        String command = pair.getFirst().toLowerCase();
        int modifier = pair.getSecond();

        switch (command) 
        {
            case "x":
                // This is a starting point. Reset the robot pose to some starting position
                
                Pose2d startingPose = new Pose2d(0, 0, new Rotation2d(0));

                Optional<Alliance> allianceColor = DriverStation.getAlliance();
                boolean isBlueAlliance = true;

                if(allianceColor.isPresent()){
                    isBlueAlliance = allianceColor.get() == Alliance.Blue;
                }

                switch (modifier) 
                {
                    case 1:
                        startingPose = isBlueAlliance ? Constants.TargetLocations.BLUE_START_LEFT : Constants.TargetLocations.RED_START_LEFT;
                        break;
                    
                    case 2:
                    default:
                        startingPose = isBlueAlliance ? Constants.TargetLocations.BLUE_START_CENTER : Constants.TargetLocations.RED_START_CENTER;
                        break;
                
                    case 3:
                        startingPose = isBlueAlliance ? Constants.TargetLocations.BLUE_START_RIGHT : Constants.TargetLocations.RED_START_RIGHT;
                        break;

                }

                m_drive.setPose(startingPose);
                
                break;

            case "a", "b", "c", "d", "e", "f", "g", "h", "i", "j", "k", "l" :
                // This is a reef position.
                // Add commands based on position and level
               /*  addCommands(
                    m_drive.pathfindThenFollowPath(DriveConstants.autoPathingConstraints,"goto_" + command),
                    new SafelyMoveToScoringPosition(m_superstructure, modifier),
                    //new WaitCommand(1.0),
                    new AutoScoreCoral(m_superstructure),
                    new GoHome(a_superstructure)
                    //new WaitCommand(1)// TODO : scoreAtLevel(level)
                    );

                    */
                break;

            case "p"  :
                // This is a left (PORT) coral station / collect position.
                // Add commands based on position and level
                /*
                addCommands(
                    Commands.parallel(                    
                        m_drive.pathfindThenFollowPath(DriveConstants.autoPathingConstraints,"goto_" + "left"+ "_" + modifier),
                        Commands.sequence(new GoHome(m_superstructure),new CollectCoral(a_superstructure))
                    )
                );
                */
                break;

            case "s" :
                // This is a right (STARBORD) coral station / collect position.
                // Add commands based on position and level
                /* 
                addCommands(
                    Commands.parallel(
                        m_drive.pathfindThenFollowPath(DriveConstants.autoPathingConstraints,"goto_" + "right"+ "_" + modifier),
                    //new WaitCommand(1) // TODO : collectFromCoralStation()
                    );*/
                break;
        
            default:
                // Invalid command
                System.out.println("Unexpected character entry: " + command + modifier);
                break;
        }
    
    }

}

private List<Pair<String, Integer>> parseInstruction(String instruction) {
    List<Pair<String, Integer>> parsedInstructions = new ArrayList<>();
    for (int i = 0; i < instruction.length(); i += 2) {
        String letter = instruction.substring(i, i + 1);
        int number = Integer.parseInt(instruction.substring(i + 1, i + 2));
        parsedInstructions.add(new Pair<>(letter, number));
    }
    return parsedInstructions;
}


}