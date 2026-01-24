package frc.robot.subsystems.intake;

import java.security.AllPermission;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

public class IntakeCommands {
    
    private IntakeCommands(){}

    public Command intakeMode(IntakeSubsystem intake){
           /*   new FunctionalCommand(
            () -> intake.setSlidePosition(intake.SLIDE_EXTENDED_POSITION),
            () -> intake.setRotatorVelocity(intake.ROTATOR_RUNNING_VELOCITY),
            () -> {
                intake.setSlidePosition(intake.SLIDE_EXTENDED_POSITION);
                intake.setRotatorVelocity(intake.ROTATOR_RUNNING_VELOCITY);
            }, 
            () -> false,
        intake);
        
         return new  FunctionalCommand(
       () -> intake.setSlidePosition(intake.SLIDE_EXTENDED_POSITION), 
       () -> intake.setRotatorVelocity(intake.ROTATOR_RUNNING_VELOCITY), 
       null, 
       null, 
       intake);
        */
       return new  FunctionalCommand(
       () -> intake.setSlidePosition(intake.SLIDE_EXTENDED_POSITION), 
       () -> intake.setRotatorVelocity(intake.ROTATOR_RUNNING_VELOCITY), 
       null, 
       () -> false, 
       intake);
        }
    }
