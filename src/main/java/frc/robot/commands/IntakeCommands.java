package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

public class IntakeCommands {
    //  public Command enterIntakeMode(IntakeSubsystem intake){ //should be default command
    //     return 
    //     new  FunctionalCommand ( //TODO rapidly extending the slide whenever an object is close to the robot is a horrible idea
    //    () -> intake.setSlidePosition(intake.SLIDE_EXTENDED_POSITION), //runs on init, extends slider
    //    () -> intake.setRollerSpeed(intake.ROLLER_VOLTS), // runs until command ends, runs roller
    //     interrupted -> intake.toRestingState(), // when the command is interrupted, return the subsystem to resting state (unexteded slider and stopped rotator)
    //    () -> !intake.intakeTargetClose() && intake.indexerTargetClose(), // stop the command when there is nothing to be picked up and nothing inside it
    //    intake // the subsystem that the command depends on
    //    );
    //     }    

}
