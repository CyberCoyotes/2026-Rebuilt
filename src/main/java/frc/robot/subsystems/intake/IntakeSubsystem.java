package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase{   

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs;

    IntakeSubsystem(IntakeIO intakeIO){
        this.io = intakeIO;
        this.inputs = new IntakeIOInputsAutoLogged();
   }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }

     //rotator methods
    public void setRotatorSpeed(double speed){
        io.setRotatorSpeed(speed);
    }

       public void stopRotator(){
        io.stopRotator();
    }

    public StatusSignal<Voltage> getRotatorVolts(){
        return io.getRotatorVolts();
    }

    //slide methods
    public void setSlidePosition(double position){
        io.setSlidePosition(position);
    }

    public StatusSignal<Angle> getSlidePosition(){
        return io.getSlidePosition();
    }
    
    //intake sensor methods
    public double getIntakeDistance(){
        return io.getIntakeDistance();
    }

     public boolean intakeTargetClose(){
        return io.indexerTargetClose();
    }

    //indexer sensor methods
    public double getIndexerDistance(){
        return io.getIndexerDistance();
    }

    //returns true if is something close to indexer TOF
    public boolean indexerTargetClose(){
        return io.indexerTargetClose();
    }

    //multi-hardware methods
    public void toRestingState(){
        io.toRestingState();
    }

    public boolean isJammed(){
        return io.isJammed();
  }



}
