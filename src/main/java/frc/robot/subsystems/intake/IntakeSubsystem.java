package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Intake;

@SuppressWarnings("unused") // Suppress warnings for unused right now

public class IntakeSubsystem extends SubsystemBase {

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs;

    // ===== State Tracking =====
    private String currentState = "IDLE";

    // ===== Intake Constants =====
    // Sensor thresholds for detecting game pieces, may need to be tuned
    final static int INTAKE_THRESHOLD = 1000; // mm, around four inches
    final static int INDEXER_THRESHOLD = 67; // mm

    final static double JAM_CURRENT_THRESHOLD = 20.0; // current should be under this
    final static double JAM_VELOCITY_THRESHOLD = 0.5; // velocity should be over this

    // Mechanical limits for the slide, may need to be tuned
    final static double SLIDE_MIN_POSITION = 0;
    final static double SLIDE_MAX_POSITION = 1.85;

    final static double SLIDE_RETRACTED_POSITION = 0;
    final static double SLIDE_BUMPER_POSITION = 0.5; // TODO Find the slide position where the slides come back to rest on the bumper but not fully retracted, up, and tucked
    final static double SLIDE_EXTENDED_POSITION = 1.80;

    final static double ROLLER_VOLTS = 4; // Voltage to run the roller at for intaking fuel, may need to be tuned
    // final static double SLIDE_VOLTS = -6; // Voltage to run the roller at for outtaking fuel, may need to be tuned

    public IntakeSubsystem(IntakeIO intakeIO) {
        this.io = intakeIO;
        this.inputs = new IntakeIOInputsAutoLogged();
    }

    @Override
    public void periodic() {
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

    //=== Low Level methods ===
    public void setRollerSpeed(double volts) {
        io.setRollerSpeed(volts);
    }

    public void runRoller() {
        io.setRollerSpeed(ROLLER_VOLTS);
    }

    public void stopRoller() {
        io.setRollerSpeed(0);
    }

    public void reverseRoller() {
        // Reverse the roller to eject fuel
        io.setRollerSpeed(-ROLLER_VOLTS);
    }

    public void setSlidePosition(double position) {
        io.setSlidePosition(position);
    }

    // Method wrapper to set the slide to the resting position
    public void retractSlides() {
        io.setSlidePosition(SLIDE_RETRACTED_POSITION);
    }

    // Method wrapper to set the slide to the extended position
    public void extendSlides() {
        io.setSlidePosition(SLIDE_EXTENDED_POSITION);
    }

    //=== Getters ===
    public double getRollerVolts() {
        return io.getRollerVolts();
    }

    public double getSlidePosition() {
        return io.getSlidePosition();
    }

    // intake sensor methods
    public double getIntakeDistance() {
        return inputs.intakeDistance;
    }

    public boolean intakeTargetClose() {
        return inputs.intakeTarget;
    }


    // multi-hardware methods
    public void toRestingState() {
        io.toRestingState();
    }

    public boolean isJammed() {
        return io.isJammed();
    }

    // ===== Commands =====

    /*
     * Probably don't need these command wrappers, but they make the button bindings
     * code cleaner and more readable, and they also allow us to easily add extra
     * logic to the commands if we want to in the future (e.g., adding a condition
     * to only extend the slides if the intake sensor detects a target)
     */
    // public Command extendSlidesCommand(){
    // return Commands.runOnce(this::extendSlides, this);

    // }

    /*
     * Probably don't need these command wrappers either, but they make the button
     * bindings code cleaner and more readable, and they also allow us to easily add
     * extra logic to the commands if we want to in the future (e.g., adding a
     * condition to only retract the slides if the intake sensor detects no target)
     */
    // public Command returnSlidesCommand(){
    // return Commands.runOnce(this::restSlides, this);
    // }

    // public Command outakeFuelCommand() {
    //     return Commands.startEnd(
    //             this::outakeFuel,
    //             this::stopRoller, this);
    // }

    public Command runRollerCommand() {
        return Commands.startEnd(
                this::runRoller,
                this::stopRoller, this);
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
        this::stopRotator, this);
  }

  public Command runRotatorCommand(){
    return Commands.startEnd(
        this::runRotator, 
        this::stopRotator, this);
  }

  public Command stopRotatorCommand(){
    return Commands.run(this::stopRotator, this);
  }

  // ===== Command Combinations =====
  /* Single command that (1) extends the slides and (2) runs the rotator.
   * Both actions are on the same subsystem, so they cannot be in a parallel
   * group — instead we call both methods inside one command.
   */

  public Command intakeFuel(){
        return Commands.startEnd(
            () -> {
                extendSlides();
                runRotator();
            },
            () -> {
                stopRotator();
            },
            this
        );
    }

    public Command stopFuelIntake(){
        return Commands.runOnce(
            () -> {
                restSlides();
                stopRotator();
            },
            this
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
            Commands.runOnce(this::stopRotator, this), // Ensure rotator is off
            returnSlidesCommand() // Retract the slides to the resting position
        );
    }

    // ===== Command Combinations =====
    /*
     * Make a parallel command sequence that
     * (1) extends the slides and stops
     * (2) runs the roller while button pressed and stops it when released
     */
    // This is the main command for intaking fuel, and will be bound to the intake
    // button on the controller
    // FIXME there were code crashes that mentioned this command
    public Command intakeFuel() {
        return Commands.startEnd(
                () -> {
                    extendSlides();
                    runRoller();
                },
                this::stopRoller,
                this);
    }

    public Command ejectFuel(){
        return Commands.startEnd(
            this::ejectFuel,
            this::stopRoller, 
            this);
    }

    // public Command stopFuelIntake() {
    //     return Commands.runOnce(
    //             // Wrap actions as Commands
    //             this::stopRoller,
    //             this::restSlides // retracts the slides
    //     // stops the roller
    //     );
    // }

    /*
     * Command to move slides from SLIDE_EXTENDED_POSITION to SLIDE_BUMPER_POSITION
     * Use MotionMagicVoltage to do this gradually
     * Intake Roller should be off
     * Effectively decrease the hopper volume to help feed fuel into the indexer and shooter
     */
    // FIXME This incomplete
    // public Command collapseHopperCommand(double duration) {
    //     return Commands.sequence(
    //             Commands.runOnce(
    //                 this::stopRoller, this), // Ensure roller is off
    //              // Retract the slides to the resting position
    //     );
    // }

}
