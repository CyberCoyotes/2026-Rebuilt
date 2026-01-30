package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

public class IntakeCommands {
    
    private IntakeCommands(){}

   /*  public Command enterIntakeMode(IntakeSubsystem intake){ //should be default command
        return 
        new  FunctionalCommand ( //TODO rapidly extending the slide whenever an object is close to the robot is a horrible idea
       () -> intake.setSlidePosition(IntakeConstants.SLIDE_EXTENDED_POSITION), //runs on init, extends slider
       () -> intake.setRotatorVelocity(IntakeConstants.ROTATOR_RUNNING_VELOCITY), // runs until command ends, runs rotator
       interrupted -> intake.toRestingState(), // when the command is interrupted, return the subsystem to resting state (unexteded slider and stopped rotator)
       () -> intake.intakeTargetClose() && intake.indexerTargetClose(), // stop the command when there is nothing to be picked up and nothing inside it
       intake // the subsystem that the command depends on
       );
        }

    public Command stopJam(IntakeSubsystem intake){
        return Commands.startEnd(() -> intake.setRotatorVelocity(-IntakeConstants.ROTATOR_MAX_VELOCITY),() -> intake.setRotatorVelocity(0), intake).withTimeout(0.5);
    }*/
    }
