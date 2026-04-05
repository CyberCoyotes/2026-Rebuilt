package frc.robot.subsystems.intake;

import java.util.function.BooleanSupplier;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;

public class IntakeSubsystem extends SubsystemBase {

    // =====================================================================
    // IO Layer
    // =====================================================================
    private final IntakeIO io;
    private final IntakeIOInputs inputs = new IntakeIOInputs();

    // =====================================================================
    // State Tracking
    // =====================================================================

    /**
     * Tracks roller direction internally so getIntakeState() doesn't need to
     * poll the IO layer on every call.
     */
    private enum RollerState {
        STOPPED, 
        RUNNING, 
        REVERSED
    }

    private RollerState rollerState = RollerState.STOPPED;

    // =====================================================================
    // Elastic Dashboard Publishers
    // =====================================================================

    // Driver-awareness data: state string, slide position, and at-target booleans.
    // Raw motor signals are captured automatically by CTRE Hoot — no need to
    // duplicate them here.
    private final NetworkTable intakeTable;
    private final StringPublisher intakeStatePublisher;
    private final DoublePublisher slidePositionPublisher;

    // =====================================================================
    // Constructor
    // =====================================================================
    public IntakeSubsystem(IntakeIO intakeIO) {
        this.io = intakeIO;

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        intakeTable = inst.getTable("Intake");
        intakeStatePublisher = intakeTable.getStringTopic("State").publish();
        slidePositionPublisher = intakeTable.getDoubleTopic("SlidePosition").publish();
    }

    // =====================================================================
    // Periodic
    // =====================================================================

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        publishTelemetry();
    }

    // Driver-facing telemetry pushed to NetworkTables for Elastic Dashboard.
    private void publishTelemetry() {
        // TODO: Consider removing these debug publishers once intake bring-up is finished.
        intakeStatePublisher.set(getIntakeState());
        slidePositionPublisher.set(inputs.slidePositionRotations);
    }

    // =====================================================================
    // STATE QUERIES  —  read-only, no side effects
    // =====================================================================

    // =====================================================================
    // State Queries
    // =====================================================================

    // Read-only helpers with no side effects.

    /** True when the roller is actively running forward (intaking). */
    public boolean isRollerRunning() {
        return rollerState == RollerState.RUNNING;
    }

    /** True when slide is within tolerance of the extended setpoint. */
    public boolean isSlideExtended() {
        return Math.abs(inputs.slidePositionRotations - Constants.Intake.SLIDE_EXTENDED_POS) < Constants.Intake.SLIDE_TOLERANCE;
    }

    /** True when slide is within tolerance of the true startup zero / hard-stop setpoint. */
    public boolean isSlideRetracted() {
        return Math.abs(inputs.slidePositionRotations - Constants.Intake.SLIDE_RETRACTED_POS) < Constants.Intake.SLIDE_TOLERANCE;
    }

    /** True when slide is within tolerance of the normal operating home/stow setpoint. */
    public boolean isSlideHome() {
        return Math.abs(inputs.slidePositionRotations - Constants.Intake.SLIDE_HOME_POS) < Constants.Intake.SLIDE_TOLERANCE;
    }

    /** True when the slide is safely past the home position for roller operation. */
    public boolean isSlidePastHome() {
        return inputs.slidePositionRotations > Constants.Intake.SLIDE_ROLLER_SAFE_POS;
    }

    // Position accessor for commands and calculations.
    public double getSlidePositionRotations() {
        return inputs.slidePositionRotations;
    }

    public String getIntakeState() {
        if (isSlideExtended() && rollerState == RollerState.RUNNING)
            return "Intaking";
        if (isSlideExtended() && rollerState == RollerState.REVERSED)
            return "Ejecting";
        if (isSlideExtended())
            return "Extended";
        if (isSlideHome())
            return "Home";
        if (isSlideRetracted())
            return "Retracted";
        return "Moving";
    }

    // =========================================================================
    // LOW-LEVEL ACTUATORS  —  called by command factories, never by each other
    // =========================================================================

    // Roller

    public void runRoller() {
        io.setRollerVoltage(Constants.Intake.ROLLER_FORWARD_VOLTS);
        rollerState = RollerState.RUNNING;
    }

    public void reverseRoller() {
        io.setRollerVoltage(Constants.Intake.ROLLER_REVERSE_VOLTS);
        rollerState = RollerState.REVERSED;
    }

    public void stopRoller() {
        io.stopRoller();
        rollerState = RollerState.STOPPED;
    }

    // ==== Slide ====
    /** Fast full extension using the default Motion Magic profile. */
    public void extendSlidesFast() {
        io.setSlidePosition(Constants.Intake.SLIDE_EXTENDED_POS);
    }

    /** Fast full retraction using the default Motion Magic profile. */
    public void retractSlidesFast() {
        io.setSlidePosition(Constants.Intake.SLIDE_HOME_POS);
    }

    /** Fast move to the normal operating home/stow setpoint using the default Motion Magic profile. */
    public void moveSlidesHome() {
        io.setSlidePosition(Constants.Intake.SLIDE_HOME_POS);
    }

    /** Graceful full retraction using the tunable slow DynamicMotionMagic profile. */
    public void retractSlidesSlow() {
        io.setSlidePositionSlow(Constants.Intake.SLIDE_RETRACTED_POS);
    }

    /** Backward-compatible alias for the default full extension mode. */
    public void extendSlides() {
        extendSlidesFast();
    }

    /** Backward-compatible alias for the default full retraction mode. */
    public void retractSlides() {
        retractSlidesFast();
    }

    public void stopSlide() {
        io.stopSlide();
    }

    /**
     * Moves slide to an arbitrary position via MotionMagic. Motor holds after.
     * Caller is responsible for clamping to valid range.
     */
    public void setSlidesToPosition(double position) {
        io.setSlidePosition(position);
    }

    /**
     * Nudges the slide by a relative amount, clamped to the allowed travel range.
     * Useful for manual operator adjustment without committing to a fixed preset.
     */
    public void nudgeSlides(double deltaRotations) {
        double target = Math.max(
                Constants.Intake.SLIDE_RETRACTED_POS,
                Math.min(
                        inputs.slidePositionRotations + deltaRotations,
                        Constants.Intake.SLIDE_MAX_POS));
        io.setSlidePosition(target);
    }

    /**
     * Retracts slide by 15 rotations from its current position, clamped to the
     * true retracted position. Used as a quick incremental jump before slow-finishing.
     */
    public void retractSlidesIncremental() {
        double target = Math.max(
                inputs.slidePositionRotations - Constants.Intake.SLIDE_INCREMENTAL_RETRACT_ROTATIONS,
                Constants.Intake.SLIDE_RETRACTED_POS);
        io.setSlidePosition(target);
    }

    // =========================================================================
    // COMMAND FACTORIES — ROLLER
    // =========================================================================

     /** Runs roller while held; stops on release. */
    public Command intakeRoller() {
        return Commands.run(this::runRoller, this)
                .finallyDo(this::stopRoller)
                .withName("IntakeRoller");
    }

    /** Reverses roller while held; stops on release. */
    public Command reverseIntakeRoller() {
        return Commands.run(this::reverseRoller, this)
                .finallyDo(this::stopRoller)
                .withName("ReverseIntakeRoller");
    }

    // =========================================================================
    // COMMAND FACTORIES — SLIDE
    // =========================================================================

      /**
     * Sends slide to the extended setpoint (runOnce — MotionMagic holds position).
     */
    public Command extendSlidesFastCmd() {
        return Commands.runOnce(this::extendSlidesFast, this)
                .withName("ExtendSlides");
    }

    /**
     * Sends slide to the retracted setpoint (runOnce — MotionMagic holds position).
     */
    public Command retractSlidesFastCmd() {
        return Commands.runOnce(this::retractSlidesFast, this)
                .withName("RetractSlides");
    }

    public Command moveSlidesHomeCmd() {
        return Commands.runOnce(this::moveSlidesHome, this)
                .withName("MoveSlidesHome");
    }

    /**
     * Sends slide to the retracted setpoint using the slow profile
     * (runOnce — MotionMagic holds position).
     */
    public Command retractSlidesSlowCmd() {
        return Commands.runOnce(this::retractSlidesSlow, this)
                .withName("RetractSlidesSlow");
    }

    /** Re-issues the slow retract profile while held so the slide keeps working toward zero. */
    public Command retractSlidesSlowHeldCmd() {
        return Commands.run(this::retractSlidesSlow, this)
                .withName("RetractSlidesSlowHeld");
    }

    /** Backward-compatible alias for the default full extension command. */
    public Command extendSlidesCmd() {
        return extendSlidesFastCmd();
    }

    /** Backward-compatible alias for the default full retraction command. */
    public Command retractSlidesCmd() {
        return retractSlidesFastCmd();
    }

    /**
     * Retracts slide by 15 rotations per call. Useful as a single button-tap
     * to nudge the slide back in stages without committing to full retraction.
     */
    public Command retractSlidesIncrementalCmd() {
        return Commands.runOnce(this::retractSlidesIncremental, this)
                .withName("RetractSlidesIncremental");
    }

    /** Repeats a small manual slide nudge while held. */
    public Command manualSlideNudgeHoldCmd(double deltaRotations) {
        return Commands.repeatingSequence(
                Commands.runOnce(() -> nudgeSlides(deltaRotations), this),
                Commands.waitSeconds(Constants.Intake.SLIDE_MANUAL_REPEAT_SECONDS))
                .withName(deltaRotations >= 0.0 ? "ManualSlideExtendHold" : "ManualSlideRetractHold");
    }

    /** Extends the slide in small repeated steps while held. */
    public Command manualSlideExtendHoldCmd() {
        return manualSlideNudgeHoldCmd(Constants.Intake.SLIDE_MANUAL_STEP_ROTATIONS);
    }

    /** Retracts the slide in small repeated steps while held. */
    public Command manualSlideRetractHoldCmd() {
        return manualSlideNudgeHoldCmd(-Constants.Intake.SLIDE_MANUAL_STEP_ROTATIONS);
    }

     // =========================================================================
    // COMMAND FACTORIES — TELEOP COMBINATIONS
    // =========================================================================

    /**
     * Primary teleop intake command. Bind to a button with whileTrue().
     *
     * While held:
     *   1. Extends slides to the full-out setpoint (completes immediately).
     *   2. Runs roller continuously.
     * On release:
     *   - Roller stops. Slides stay extended (MotionMagic holds them).
     *
     * Note: extendSlidesFastCmd() (runOnce) is used rather than calling extendSlidesFast()
     * directly inside a lambda — the runOnce approach was more reliable on hardware.
     */
    public Command intakeFuel() {
        return Commands.sequence(
                extendSlidesFastCmd(),
                Commands.run(this::runRoller, this)
                        .finallyDo(this::stopRoller))
                .withName("IntakeFuel");
    }

    /**
     * Stops roller and retracts slides in a single action.
     * Bind to a button with onTrue() to cancel intakeFuel() and stow the intake.
     */
    public Command stopFuel() {
        return Commands.runOnce(
                () -> {
                    stopRoller();
                    retractSlidesFast();
                }, this)
                .withName("StopFuel");
    }

    /**
     * Timed compress — slowly retracts slides using the slow DynamicMotionMagic
     * profile while running the roller to compact fuel into the hopper.
     *
     * Ends automatically after {@code timeoutSeconds}. Good for a timed button
     * press where you want the action to finish on its own.
     *
     * Typical usage:
     * <pre>
     *   operator.leftBumper().onTrue(intake.compressFuel(2.0));
     * </pre>
     *
     * @param timeoutSeconds How long to run (~2.0 s matches typical slide travel).
     */
    public Command compressFuel(double timeoutSeconds) {
        return Commands.run(
                () -> {
                    retractSlidesSlow();
                    runRoller();
                }, this)
                .withTimeout(timeoutSeconds)
                .finallyDo(() -> stopRoller())
                .withName("CompressFuel");
    }
     
    /**
     * Button-held compress — same slow retraction + roller as compressFuel(),
     * but runs only while the button is held and stops the instant it's released.
     *
     * Use this when you want manual control over how long compression runs.
     * Use compressFuel(seconds) when you want it to self-terminate.
     *
     * Typical usage:
     *   operator.leftBumper().whileTrue(intake.compressFuelHeld());
     */
    public Command fuelPumpSlow() {
        return Commands.runEnd(
                () -> {
                    retractSlidesSlow();
                    runRoller();
                },
                this::stopRoller,
                this)
                .withName("FuelPumpSlow");
    }

    /**
     * Backward-compatible alias for the slow fuel compression/pump command.
     * Runs only while held and stops immediately on release.
     */
    public Command compressFuelHeld() {
        return fuelPumpSlow()
                .withName("CompressFuelHeld");
    }

    /**
     * Workaround: multi-step slide retraction that re-sends the retract setpoint
     * in stages with pauses between each attempt.
     *
     * Background: MotionMagic occasionally fails to complete retraction in a single
     * runOnce call under load. This command re-commands the retracted setpoint four
     * times with 1-second gaps, giving the mechanism time to settle after each push.
     *
     * Status: Still in use as a fallback. If MotionMagic retraction becomes reliable
     * under all conditions, this command can be removed in favor of retractSlidesFastCmd().
     */
    public Command retractSlidesStack() {
        return Commands.sequence(
                Commands.runOnce(this::retractSlidesFast, this),
                Commands.waitSeconds(1),
                Commands.runOnce(this::retractSlidesFast, this),
                Commands.waitSeconds(1),
                Commands.runOnce(this::retractSlidesFast, this),
                Commands.waitSeconds(1),
                Commands.runOnce(this::retractSlidesFast, this))
                .withName("RetractSlidesStack");
    }

    // =========================================================================
    // COMMAND FACTORIES — Fuel Pump
    // =========================================================================

    /* More involved and current default of agitating the fuel during shooting */
    public Command fuelPump() {
        Timer bounceTimer = new Timer();
        return Commands.run(() -> {
            runRoller();
            if (bounceTimer.get() < 0.5) {
                setSlidesToPosition(Constants.Intake.SLIDE_PUMP_OUT_POS);
            } else if (bounceTimer.get() < 1.0) {
                setSlidesToPosition(Constants.Intake.SLIDE_PUMP_IN_POS);
            } else {
                bounceTimer.reset();
            }
        }, this)
                .beforeStarting(() -> {
                    bounceTimer.reset();
                    bounceTimer.start();
                })
                .finallyDo(() -> stopRoller())
                .withName("FuelPump");
    }

    /**
     * Continuously cycles the slides between SLIDE_BOUNCE_DOWN_POS and SLIDE_BOUNCE_UP_POS
     * while running the roller, for as long as the button is held.
     *
     * Use with whileTrue() — the command runs indefinitely and stops cleanly on release.
     * Replaces fuelPumpBasic().repeatedly() which had roller-stop gaps between cycles.
     */
    public Command fuelPumpCycle() {
        Timer cycleTimer = new Timer();
        return Commands.run(() -> {
            runRoller();
            double t = cycleTimer.get();
            if (t < 0.5) {
                setSlidesToPosition(Constants.Intake.SLIDE_PUMP_OUT_POS);
            } else if (t < 1.0) {
                setSlidesToPosition(Constants.Intake.SLIDE_PUMP_IN_POS);
            } else {
                cycleTimer.restart(); // reset + start, so next cycle begins immediately
            }
        }, this)
                .beforeStarting(cycleTimer::restart)
                .finallyDo(this::stopRoller)
                .withName("FuelPumpCycle");
    }

    /**
     * Runs the fuel pump cycle (bouncing slides + roller) for a fixed duration.
     * Ends naturally after {@code seconds}, making it safe for autonomous and
     * Choreo event linking via {@code trajectory.atTime("FuelPump").onTrue(...)}.
     *
     * @param seconds How long to run the pump cycle.
     */
    public Command fuelPumpCycleAuto(double seconds) {
        Timer cycleTimer = new Timer();
        return Commands.run(() -> {
            runRoller();
            double t = cycleTimer.get();
            if (t < 0.5) {
                setSlidesToPosition(Constants.Intake.SLIDE_PUMP_OUT_POS);
            } else if (t < 1.0) {
                setSlidesToPosition(Constants.Intake.SLIDE_PUMP_IN_POS);
            } else {
                cycleTimer.restart();
            }
        }, this)
                .beforeStarting(cycleTimer::restart)
                .withTimeout(seconds)
                .finallyDo(this::stopRoller)
                .withName("FuelPumpCycleAuto");
    }

    // Loopable and repeatable version of fuelPump() for more manual control over timing and cycles.
    public Command fuelPumpBasic() {
        return Commands.sequence(
                Commands.run(() -> {
                    runRoller();
                    setSlidesToPosition(Constants.Intake.SLIDE_PUMP_OUT_POS);
                }, this)
                        .withTimeout(0.5),
                Commands.run(() -> {
                    runRoller();
                    setSlidesToPosition(Constants.Intake.SLIDE_PUMP_IN_POS);
                }, this)
                        .withTimeout(0.5))
                .finallyDo(this::stopRoller).withName("FuelPumpBasic");
    }

    // Ideally set up to take an argument for number of cycles, but for now just a quick test of multiple repeats of the basic bounce sequence.
    public Command fuelPumpSetCycles() {
        return Commands.sequence(
                // ==== Cycle 1 ====
                Commands.run(() -> {
                    runRoller();
                    setSlidesToPosition(Constants.Intake.SLIDE_PUMP_OUT_POS);
                }, this)
                        .withTimeout(0.5),
                // Short pause between down and up to allow fuel to settle before bouncing back up again
                Commands.waitSeconds(0.1),
                Commands.run(() -> {
                    runRoller();
                    setSlidesToPosition(Constants.Intake.SLIDE_PUMP_IN_POS);
                }, this)
                    .withTimeout(0.5),
                Commands.waitSeconds(0.1),

                // ==== Cycle 2 ====
                Commands.run(() -> {
                    runRoller();
                    setSlidesToPosition(Constants.Intake.SLIDE_PUMP_OUT_POS);
                }, this)
                        .withTimeout(0.5),
                Commands.waitSeconds(0.1),
                Commands.run(() -> {
                    runRoller();
                    setSlidesToPosition(Constants.Intake.SLIDE_PUMP_IN_POS);
                }, this)
                        .withTimeout(0.5),
                Commands.waitSeconds(0.1)


                .finallyDo(this::stopRoller).withName("FuelPumpBasic"));
    }

    // Notable that this one is missing roller commands
    public Command fuelPumpSetCyclesRetract() {
        return Commands.sequence(
                // ==== Cycle 1 ====
                Commands.runOnce(() -> setSlidesToPosition(Constants.Intake.SLIDE_PUMP_OUT_POS), this),
                Commands.waitSeconds(0.5),
                Commands.runOnce(() -> setSlidesToPosition(Constants.Intake.SLIDE_PUMP_IN_POS), this),
                Commands.waitSeconds(0.5),
                // ==== Cycle 2 ====
                Commands.runOnce(() -> setSlidesToPosition(Constants.Intake.SLIDE_PUMP_OUT_POS), this),
                Commands.waitSeconds(0.5),
                Commands.runOnce(() -> setSlidesToPosition(Constants.Intake.SLIDE_PUMP_IN_POS), this),
                Commands.waitSeconds(0.5),
                // ==== Cycle 3 ====
                Commands.runOnce(() -> setSlidesToPosition(Constants.Intake.SLIDE_HOME_POS), this))
                .withName("FuelPumpThenRetractSlide");
    }
    // =========================================================================
    // COMMAND FACTORIES — AUTONOMOUS
    // =========================================================================

    /**
     * Auton roller — runs for a fixed number of seconds.
     * Shorthand for intakeRoller().withTimeout(seconds).
     *
     * @param seconds Duration to run the roller.
     */
    public Command intakeRollerTimer(double seconds) {
        return intakeRoller().withTimeout(seconds)
                .withName("IntakeRollerTimer");
    }

    /**
     * Auton roller — runs until a sensor condition is met (e.g. hopper full).
     *
     * @param condition BooleanSupplier that returns true when the roller should stop.
     */
    public Command intakeRollerUntil(BooleanSupplier condition) {
        return intakeRoller().until(condition)
                .withName("IntakeRollerUntil");
    }

    /**
     * Full auton intake sequence.
     *
     * Flow:
     *   1. Extends slides to the MotionMagic setpoint (runOnce).
     *   2. Runs roller for {@code intakeTimeout} seconds.
     *   3. Stops roller on completion or interruption.
     *
     * Slides remain extended after the command — call stopFuel() or
     * retractSlidesFastCmd() afterward if stowing is needed.
     *
     * @param intakeTimeout How long to run the roller after extending.
     */
    public Command intakeFuelTimer(double intakeTimeout) {
        return Commands.sequence(
                extendSlidesFastCmd(),
                Commands.run(this::runRoller, this)
                        .withTimeout(intakeTimeout)
                        .finallyDo(this::stopRoller))
                .withName("IntakeFuelTimer");
    }

    public Command intakeFuelUntil(BooleanSupplier condition) {
        return Commands.sequence(
                extendSlidesFastCmd(),
                Commands.run(this::runRoller, this)
                        .until(condition)
                        .finallyDo(this::stopRoller))
                .withName("IntakeFuelUntil");
    }

    /**
     * Two-phase retraction with roller running — designed to be used as the
     * non-deadline side of a Commands.deadline() alongside a shoot sequence.
     *
     * Phase 1 (runOnce): Quick 15-rotation jump back via normal MotionMagic.
     *                    Roller starts here.
     * Phase 2 (run loop): Slow DynamicMotionMagic finish to the bumper/stow pose
     *                     while the roller keeps running.
     *
     * Roller is stopped in finallyDo so any interrupt (e.g. deadline finishing)
        * still cleans up properly.
        *
     * Typical auton usage:
     *   Commands.deadline(
     *       FuelCommands.shootWithPreset(shooter, indexer, rpm, hood),
     *       intake.retractSlidesWithRollerCmd());
     */
    public Command retractSlidesAuton() {
        return Commands.sequence(
                Commands.runOnce(() -> {
                    retractSlidesIncremental();
                    runRoller();
                }, this),
                Commands.run(() -> {
                    retractSlidesSlow();
                    runRoller();
                }, this).until(this::isSlideRetracted))
                .finallyDo(() -> stopRoller())
                .withName("RetractSlidesWithRoller");
    }

} // end of class
