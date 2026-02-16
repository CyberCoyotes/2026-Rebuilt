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

    final static double JAM_CURRENT_THRESHOLD = 20.0; // current should be under this
    // final static double JAM_VELOCITY_THRESHOLD = 0.5; // velocity should be over this

    // Mechanical limits for the slide, may need to be tuned
    final static double SLIDE_MIN_POSITION = 0;
    final static double SLIDE_MAX_POSITION = 1.91;

    final static double SLIDE_RETRACTED_POSITION = 0.25;
    final static double SLIDE_EXTENDED_POSITION = 1.85;

    final static double ROLLER_VOLTS = 8; // Voltage to run the roller at for intaking fuel, may need to be tuned
    // 4 is not enough

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

    public boolean isJammed() {
        return false; //io.isJammed();
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

    public Command stopRollerCommand() {
        return Commands.run(this::stopRoller, this);
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

    public Command stopFuel() {
        return Commands.runOnce(
                // Wrap actions as a single Runnable command
                () -> {
                    this.stopRoller();
                    this.retractSlides();
                },
                this
        );
    }

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
