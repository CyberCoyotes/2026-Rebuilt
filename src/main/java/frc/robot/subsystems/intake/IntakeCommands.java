package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

public class IntakeCommands {
     public Command enterIntakeMode(IntakeSubsystem intake){ //should be default command
        return 
        new  FunctionalCommand ( //TODO rapidly extending the slide whenever an object is close to the robot is a horrible idea
       () -> intake.setSlidePosition(IntakeConstants.SLIDE_EXTENDED_POSITION), //runs on init, extends slider
       () -> intake.setRotatorSpeed(IntakeConstants.ROTATOR_RUNNING_VELOCITY), // runs until command ends, runs rotator
        interrupted -> intake.toRestingState(), // when the command is interrupted, return the subsystem to resting state (unexteded slider and stopped rotator)
       () -> !intake.intakeTargetClose() && intake.indexerTargetClose(), // stop the command when there is nothing to be picked up and nothing inside it
       intake // the subsystem that the command depends on
       );
        }

    /* FIXME
    public Command intakeFuelWithSubCommands(IntakeSubsystem intake){
        // Extend slides and run rotator in a single command (same subsystem, cannot use parallel)
        return Commands.startEnd(
            () -> {
                intake.extendSlides();
                intake.runRotator();
            },
            () -> {
                intake.stopRotator();
            },
            intake
        );
    }
    */

/* FIXME    
    public Command intakeFuel(IntakeSubsystem intake){
        // Extend slides and run rotator in a single command (same subsystem, cannot use parallel)
        return Commands.startEnd(
            () -> {
                intake.extendSlides();
                intake.runRotator();
            },
            () -> {
                intake.stopRotator();
            },
            intake
        );
    }

    public Command stopFuelIntake(IntakeSubsystem intake){
        // Retract slides and stop rotator in a single command (same subsystem, cannot use parallel)
        return Commands.runOnce(
            () -> {
                intake.restSlides();
                intake.stopRotator();
            },
            intake
        );
    }
    */

    public Command outakeFuel(IntakeSubsystem intake){
        return Commands.startEnd(
            () -> intake.setRotatorSpeed(-IntakeConstants.ROTATOR_MAX_VELOCITY),
            () -> intake.setRotatorSpeed(0), intake);
    }

}
