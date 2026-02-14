package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Intake;


@SuppressWarnings("unused") // Suppress warnings for unused right now

public class IntakeSubsystem extends SubsystemBase{   

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs;

    // ===== State Tracking =====
    private String currentState = "IDLE"; 

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

     //rotator methods
    public void setRotatorSpeed(double speed){
        io.setRotatorSpeed(speed);
    }

    public void runRotator(){
        io.setRotatorSpeed(IntakeConstants.ROTATOR_RUNNING_VELOCITY);
    }
    
    public void stopRotator(){
        io.stopRotator();
    }

    public double getRotatorVolts(){
        return io.getRotatorVolts();
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



}
