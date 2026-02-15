package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Intake;


@SuppressWarnings("unused") // Suppress warnings for unused right now

public class IntakeSubsystem extends SubsystemBase{   

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs;

    // ===== State Tracking =====
    private String currentState = "IDLE"; 

    // ===== Intake Constants =====
    final static int INTAKE_THRESHOLD = 1000; //mm, around four inches

     final static int INDEXER_THRESHOLD = 67; //mm

     final static double JAM_CURRENT_THRESHOLD = 20.0; // current should be under this
     final static double JAM_VELOCITY_THRESHOLD = 0.5; // velocity should be over this


     final static double SLIDE_EXTENDED_POSITION = 1.85; // Updated to true values 2/13/26
     final static double SLIDE_RESTING_POSITION = 0;
     final static double ROLLER_VOLTAGE = 4; // Voltage to run the roller at for intaking fuel, may need to be tuned    

    public IntakeSubsystem(IntakeIO intakeIO){
        this.io = intakeIO;
        this.inputs = new IntakeIOInputsAutoLogged();
   }

       @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }
    /**
     * Sets the current state for dashboard display.
     *
     * @param state State string (e.g., "IDLE", "INTAKING", "EXTENDED")
     */
    public void setState(String state) {
        this.currentState = state;
    }

     //roller methods
    public void setRollerSpeed(double speed){
        io.setRollerSpeed(speed);
    }

    public void runRoller(){
        io.setRollerSpeed(IntakeConstants.ROTATOR_RUNNING_VELOCITY);
    }
    
    public void stopRoller(){
        io.setRollerSpeed(0);
    }

    public void outakeFuel(){
        // Reverse the roller to eject fuel
        io.setRollerSpeed(-IntakeConstants.ROTATOR_RUNNING_VELOCITY);
    } 

    public double getRollerVolts(){
        return io.getRollerVolts();
    }

    //slide methods
    public void setSlidePosition(double position){
        io.setSlidePosition(position);
    }

    // Method wrapper to set the slide to the resting position
    public void restSlides(){
        io.setSlidePosition(IntakeConstants.SLIDE_RESTING_POSITION);
    }

    // Method wrapper to set the slide to the extended position
    public void extendSlides(){
        io.setSlidePosition(IntakeConstants.SLIDE_EXTENDED_POSITION);
    }

    public double getSlidePosition(){
        return io.getSlidePosition();
    }
    
    //intake sensor methods
    public double getIntakeDistance(){
        return inputs.intakeDistance;
    }

     public boolean intakeTargetClose(){
        return inputs.intakeTarget;
    }

    //indexer sensor methods
    public double getIndexerDistance(){
        return inputs.indexerDistance;
    }

    //returns true if is something close to indexer TOF
    public boolean indexerTargetClose(){
        return inputs.indexerTarget;
    }

    //multi-hardware methods
    public void toRestingState(){
        io.toRestingState();
    }

    public boolean isJammed(){
        return io.isJammed();
  }

  // ===== Commands =====
  public Command extendSlidesCommand(){
    return Commands.runOnce(this::extendSlides, this);
    
  }

  public Command returnSlidesCommand(){
    return Commands.runOnce(this::restSlides, this);
  }

  // Return slides to resting position over a set amount of time (e.g., 2 second) to help feed fuel into the indexer and shooter
  public Command returnSlidesCommandSlowyly(){
        final double duration = 2.0; // seconds to move to resting position
        final double[] startPos = new double[1];
        final edu.wpi.first.wpilibj.Timer timer = new edu.wpi.first.wpilibj.Timer();

        return Commands.sequence(
            Commands.runOnce(() -> {
                startPos[0] = getSlidePosition();
                timer.reset();
                timer.start();
            }, this),
            Commands.run(() -> {
                double progress = Math.min(1.0, timer.get() / duration);
                double target = IntakeConstants.SLIDE_RESTING_POSITION;
                double position = startPos[0] + (target - startPos[0]) * progress;
                setSlidePosition(position);
            }, this).withTimeout(duration),
            Commands.runOnce(() -> {
                timer.stop();
                setSlidePosition(IntakeConstants.SLIDE_RESTING_POSITION);
            }, this)
        );
    }

  public Command outakeFuelCommand(){
    return Commands.startEnd(
        this::outakeFuel,
        this::stopRoller, this);
  }

  public Command runRollerCommand(){
    return Commands.startEnd(
        this::runRoller, 
        this::stopRoller, this);
  }

  public Command stopRollerCommand(){
    return Commands.run(this::stopRoller, this);
  }

  // ===== Command Combinations =====
  /* Make a parallel command sequence that 
   * (1) extends the slides and stops
   * (2) runs the roller while button pressed and stops it when released
   */
  // This is the main command for intaking fuel, and will be bound to the intake button on the controller
  // FIXME there were code crashes that mentioned this command
  public Command intakeFuel(){
        return Commands.parallel(
            extendSlidesCommand(), // extends the slides
            runRollerCommand()
        );
    }

    public Command stopFuelIntake(){
        return Commands.parallel(
            // Wrap actions as Commands
            returnSlidesCommand(), // retracts the slides
            stopRollerCommand() // stops the roller
        );
    }


    /* 
    Simple command to bring in the slides to the home-resting position over a set amount of time
    Intake Roller is off
    Intention is to "collapse" the hopper and with the slide position coming to rest, 
    decrease the hopper volume to help feed fuel into the indexer and shooter
    */ 
    public Command collapseHopperCommand(double duration){
        return Commands.sequence(
            Commands.runOnce(this::stopRoller, this), // Ensure roller is off
            returnSlidesCommand() // Retract the slides to the resting position
        );
    }


}
