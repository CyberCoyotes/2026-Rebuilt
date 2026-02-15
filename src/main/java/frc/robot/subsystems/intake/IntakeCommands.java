package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

public class IntakeCommands {
     public Command enterIntakeMode(IntakeSubsystem intake){ //should be default command
        return 
        new  FunctionalCommand ( //TODO rapidly extending the slide whenever an object is close to the robot is a horrible idea
       () -> intake.setSlidePosition(IntakeConstants.SLIDE_EXTENDED_POSITION), //runs on init, extends slider
       () -> intake.setRollerSpeed(IntakeConstants.ROTATOR_RUNNING_VELOCITY), // runs until command ends, runs roller
        interrupted -> intake.toRestingState(), // when the command is interrupted, return the subsystem to resting state (unexteded slider and stopped rotator)
       () -> !intake.intakeTargetClose() && intake.indexerTargetClose(), // stop the command when there is nothing to be picked up and nothing inside it
       intake // the subsystem that the command depends on
       );
        }

    /* FIXME
    public Command intakeFuelWithSubCommands(IntakeSubsystem intake){
        // Make a parallel command that (1) extends the slides and (2) runs the rotator at the same time
        return Commands.parallel(
            // Wrap actions as Commands
            Commands.runOnce(() -> intake.extendSlides(), intake), // extends the slides
            // Commands.runOnce(intake::extendSlides, intake), // Functionally equivalent to the above line
            Commands.run(() -> intake.runRotator(), intake) // runs the rotator
            // Commands.run(intake::runRotator, intake) // Functionally equivalent to the above line
        );
    }
    */

/* FIXME    
    public Command intakeFuel(IntakeSubsystem intake){
        // Make a parallel command that (1) extends the slides and (2) runs the rotator at the same time
        return Commands.parallel(
            intake.extendSlidesCommand(), // extends the slides
            intake.runRotatorCommand() // runs the rotator
        );
    }

    public Command stopFuelIntake(IntakeSubsystem intake){
        // Make a parallel command that (1) retracts the slides and (2) stops the rotator at the same time
        return Commands.parallel(
            // Wrap actions as Commands
            Commands.runOnce(() -> intake.restSlides(), intake), // retracts the slides
            Commands.run(() -> intake.stopRotator(), intake) // stops the rotator
        );
    }
    */

    // FIXME this seems outakeFuel logic seems messed up, double check
    public Command outakeFuel(IntakeSubsystem intake){
        return Commands.startEnd(
            () -> intake.setRollerSpeed(-IntakeConstants.ROTATOR_MAX_VELOCITY),
            () -> intake.setRollerSpeed(0), intake);
    }

    

}
