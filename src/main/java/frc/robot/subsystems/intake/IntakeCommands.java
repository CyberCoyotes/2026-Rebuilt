package frc.robot.subsystems.intake;

import java.security.AllPermission;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

public class IntakeCommands {
    
    private IntakeCommands(){}

    public Command enterIntakeMode(IntakeSubsystem intake){
        return 
        new  FunctionalCommand (
       () -> intake.setSlidePosition(intake.SLIDE_EXTENDED_POSITION), //runs on init, extends slider
       () -> intake.setRotatorVelocity(intake.ROTATOR_RUNNING_VELOCITY), // runs until command ends, runs rotator
       interrupted -> intake.toRestingState(), // when the command is interrupted, return the subsystem to resting state (unexteded slider and stopped rotator)
       () -> intake.intakeTargetClose() && intake.indexerTargetClose(), // stop the command when there is nothing to be picked up and nothing inside it
       intake // the subsystem that the command depends on
       );
        }
    }
