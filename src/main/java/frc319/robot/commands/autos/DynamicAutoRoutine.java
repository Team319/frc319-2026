// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc319.robot.commands.autos;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc319.lib.util.AllianceFlipUtil;
import frc319.robot.Constants.DriveConstants;
import frc319.robot.commands.DriveCommands;
import frc319.robot.FieldConstants;
import frc319.robot.Robot;
import frc319.robot.RobotContainer.RobotStates;
import frc319.robot.subsystems.drive.Drive;


/** Add your docs here. */
public class DynamicAutoRoutine extends SequentialCommandGroup {

    String m_instruction = "";
    Drive m_drive;

    public class AutoConstants {
        public static class TargetLocations {

            // These are all for blue origin, blue alliance case. We apply a flip when used for red alliance. 

            public static final Translation2d DRIVETRAIN_LEFT_TRENCH = new Translation2d(FieldConstants.LinesVertical.starting, 7.5 ); // we can grab these from pathplanner too...
            public static final Translation2d DRIVETRAIN_LEFT_BUMP = new Translation2d(FieldConstants.LinesVertical.starting, 5.5);
            
            public static final Translation2d CENTER_HUB = new Translation2d(FieldConstants.LinesVertical.starting, FieldConstants.LinesHorizontal.center);

            public static final Translation2d RIGHT_BUMP = new Translation2d(FieldConstants.LinesVertical.starting, 2.5);
            public static final Translation2d RIGHT_TRENCH = new Translation2d(FieldConstants.LinesVertical.starting, 0.575);
            
        }
    }
    
    private double startingHeadingDegrees = 0.0;

    public DynamicAutoRoutine(Drive a_drive) {
        this(a_drive, SmartDashboard.getString("DynamicAutoInput", ""));
    }

    public DynamicAutoRoutine(Drive a_drive, String instruction) {

        if(!DriverStation.isEnabled()){
            return; // don't want to actually run when initializing... (so we don't mess up the starting pose)
        }

        // Constructor
        m_drive = a_drive;
        //Populate some string from Dashboard with format <ReefPosition><ReefLevel>..." ( ie - A1B2C3D4 ; A4B4C4D4 ; etc )
        m_instruction = instruction;

        if (m_instruction == null || m_instruction.isEmpty()) {

            System.out.println("request is empty or unexpected... instruction = " + m_instruction);
            m_instruction = "";
        }

        List<Pair<String, Integer>> parsedInstructions = parseInstruction(m_instruction);
        // TODO : Break down instruction

        // Reset robot pose to starting position.

        Optional<Alliance> allianceColor = DriverStation.getAlliance();
        boolean isBlueAlliance = true;

        if(allianceColor.isPresent()){
            isBlueAlliance = allianceColor.get() == Alliance.Blue;
        }

        for (Pair<String, Integer> pair : parsedInstructions) {
            String command = pair.getFirst();//.toLowerCase();  Using upper case because multiple L and Rs
            int modifier = pair.getSecond();

            double waitBeforeStateTime = 1.0;

            String pathToFollow = "";

            switch (command) 
            {
                case "h":

                    // This is a starting heading to seed into the pigeon at the start of auto
                    switch (modifier) 
                    {
                        case 0: // North
                            startingHeadingDegrees = isBlueAlliance ? 0.0 : 180.0;
                            break;
                        
                        case 1: // South
                        
                            startingHeadingDegrees = isBlueAlliance ? 180.0 : 0.0;
                            break;

                        case 2: //East
                            startingHeadingDegrees = isBlueAlliance ? 90.0 : 270.0;
                            break;

                        case 3: //West
                            startingHeadingDegrees = isBlueAlliance ? 270.0 : 90.0;
                            break;
                    }

                    //for (int i = 0; i < 10; i++) {
                        //m_drive.resetGyro();
                        m_drive.setHeading(startingHeadingDegrees);
                    //}
                    
                    break;

                case "x":
                    // This is a starting point. Reset the robot pose to some starting position
                    
                    Translation2d startingPose = new Translation2d(0, 0);


                    switch (modifier) 
                    {
                        case 1:
                            startingPose = AutoConstants.TargetLocations.DRIVETRAIN_LEFT_TRENCH;
                            break;
                        
                        case 2:
                            startingPose = AutoConstants.TargetLocations.DRIVETRAIN_LEFT_BUMP;
                            break;
                    
                        case 3:
                            startingPose = AutoConstants.TargetLocations.CENTER_HUB;
                            break;
                        
                        case 4:
                            startingPose = AutoConstants.TargetLocations.RIGHT_BUMP;
                            break;

                        case 5:
                            startingPose = AutoConstants.TargetLocations.RIGHT_TRENCH;
                            break;

                    }

                    startingPose = AllianceFlipUtil.apply(startingPose);
                    Pose2d startingPoseWithHeading = new Pose2d(startingPose, Rotation2d.fromDegrees(startingHeadingDegrees));
                    m_drive.setPose(startingPoseWithHeading);
                    
                    break;

                // Go "and" do something 
                // 0 is nothing
                // 1 is collect
                // 2 is shoot
                // 3 is collect and shoot
                case "L":
                    pathToFollow = "go_left";
                    dynamicAction(pathToFollow, modifier, waitBeforeStateTime);
                    break;

                case "l":
                    pathToFollow = "go_left_spike";
                    dynamicAction(pathToFollow, modifier, waitBeforeStateTime);
                    break;

                case "R":
                    pathToFollow = "go_left";
                    dynamicAction(pathToFollow, modifier, waitBeforeStateTime, true);
                    break;
                    

                case "r":
                    pathToFollow = "go_left_spike";
                    dynamicAction(pathToFollow, modifier, waitBeforeStateTime, true);
                    break;
                                        
                // Go To Depot and do something
                case "d":
                    pathToFollow = "go_depot";
                    dynamicAction(pathToFollow, modifier, waitBeforeStateTime);
                    break;

                // Go To Outpost and do something
                case "o":
                    pathToFollow = "go_right_outpost";
                    dynamicAction(pathToFollow, modifier, waitBeforeStateTime);
                    break;

                case "P":
                    pathToFollow = "go_left_fast";
                    dynamicChoreoAction(pathToFollow, modifier, waitBeforeStateTime, false);
                    break;

                case "p":
                    pathToFollow = "go_left_spike_fast";
                    dynamicChoreoAction(pathToFollow, modifier, waitBeforeStateTime, false);
                    break;

                case "Q":
                    pathToFollow = "go_left_fast";
                    dynamicChoreoAction(pathToFollow, modifier, waitBeforeStateTime, true);
                    break;

                case "q":
                    pathToFollow = "go_left_spike_fast";
                    dynamicChoreoAction(pathToFollow, modifier, waitBeforeStateTime, true);
                    break;

                case "O":
                    pathToFollow = "go_right_from_outpost";
                    dynamicChoreoAction(pathToFollow, modifier, waitBeforeStateTime);
                    break;

                // Shoot for Time n seconds
                case "s":
                    addCommands(
                        new InstantCommand( 
                            ()-> Robot.m_robotContainer.setRobotState(RobotStates.SHOOTING_AUTO))
                                .withDeadline(new WaitCommand(modifier)
                        ).andThen(new InstantCommand( 
                                    ()-> Robot.m_robotContainer.setRobotState(RobotStates.STOP_SHOOTING)))
                    );
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



    private void dynamicAction(String pathToFollow, int modifier, double waitBeforeStateTime, boolean mirrorPath){

        Command commandToAdd = new InstantCommand(); // default command if we don't match any cases is just to wait for a bit (so we have time to set up the robot and then it will just sit there)

        switch(modifier){
        case 0:
            commandToAdd = new InstantCommand( 
                    ()-> Robot.m_robotContainer.setRobotState(RobotStates.STOWED));
            
            break;
        case 1:
            commandToAdd = new InstantCommand( 
                    ()-> Robot.m_robotContainer.setRobotState(RobotStates.COLLECTING));
            break;


        case 2:
            commandToAdd = new InstantCommand( 
                    ()-> Robot.m_robotContainer.setRobotState(RobotStates.SHOOTING));
            break;
        case 3:
            commandToAdd = new InstantCommand( 
                    ()-> Robot.m_robotContainer.setRobotState(RobotStates.SNOWBLOW));
            break;

        }

        addCommands( new WaitCommand(waitBeforeStateTime).andThen(commandToAdd)
                            .withDeadline(DriveCommands.pathfindThenFollowPath(
                                pathToFollow, mirrorPath
                        ).andThen(new InstantCommand( 
                        ()-> Robot.m_robotContainer.setRobotState(RobotStates.IDLE)))
                        )
                    );

    }

    private void dynamicAction(String pathToFollow, int modifier, double waitBeforeStateTime){
        dynamicAction(pathToFollow, modifier, waitBeforeStateTime, false);
    }

    private void dynamicChoreoAction(String pathToFollow, int modifier, double waitBeforeStateTime, boolean mirrorPath){

        Command commandToAdd = new InstantCommand(); // default command if we don't match any cases is just to wait for a bit (so we have time to set up the robot and then it will just sit there)

        switch(modifier){
        case 0:
            commandToAdd = new InstantCommand( 
                    ()-> Robot.m_robotContainer.setRobotState(RobotStates.STOWED));
            
            break;
        case 1:
            commandToAdd = new InstantCommand( 
                    ()-> Robot.m_robotContainer.setRobotState(RobotStates.COLLECTING));
            break;


        case 2:
            commandToAdd = new InstantCommand( 
                    ()-> Robot.m_robotContainer.setRobotState(RobotStates.SHOOTING));
            break;
        case 3:
            commandToAdd = new InstantCommand( 
                    ()-> Robot.m_robotContainer.setRobotState(RobotStates.SNOWBLOW));
            break;

        }

        addCommands( new WaitCommand(waitBeforeStateTime).andThen(commandToAdd)
                            .withDeadline(DriveCommands.pathfindThenFollowChoreoPath(
                                pathToFollow, mirrorPath
                        ).andThen(new InstantCommand( 
                        ()-> Robot.m_robotContainer.setRobotState(RobotStates.IDLE)))
                        )
                    );

    }

    private void dynamicChoreoAction(String pathToFollow, int modifier, double waitBeforeStateTime){
        dynamicChoreoAction(pathToFollow, modifier, waitBeforeStateTime, false);
    }

}