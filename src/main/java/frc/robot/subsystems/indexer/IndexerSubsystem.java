package frc.robot.subsystems.indexer;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.indexer.IndexerIO.IndexerIOInputs;

public class IndexerSubsystem extends SubsystemBase {

    // =====================================================================
    // Indexer State
    // =====================================================================
    /**
     * IndexerState — Tracks what the indexer is currently doing.
     *
     * Using an enum instead of Strings means a typo is a compile error,
     * not a silent bug. Compare:
     * "FEEDNG" (compiles, breaks logic silently)
     * IndexerState.FEEDNG (won't compile — caught immediately)
     */
    public enum IndexerState {
        IDLE,
        FEEDING,
        EJECTING,
        SENDING_FUEL
    }


    // =====================================================================
    // IO Layer
    // =====================================================================
    private final IndexerIO io;

    private final IndexerIOInputs inputs = new IndexerIOInputs();

    // =====================================================================
    // State
    // =====================================================================
    private IndexerState currentState = IndexerState.IDLE;

    private boolean isFuelDetected = false;
    /** True once fuel has been seen at least once since last reset — guards donePassingFuel(). */
    private boolean hasSeenFuel = false;

    private double lastDetectionTimestamp = -1.0;
    private double secondsSinceLastDetection = Double.POSITIVE_INFINITY;

    // =====================================================================
    // Elastic Dashboard Publishers
    // =====================================================================
    private final NetworkTable indexerTable;
    private final BooleanPublisher chuteDetectedPublisher;
    private final DoublePublisher chuteDistancePublisher;
    private final StringPublisher fuelStatusColorPublisher;
    private final BooleanPublisher chuteEmptyPublisher;

    // =====================================================================
    // Constructor
    // =====================================================================
    public IndexerSubsystem(IndexerIO io) {
        this.io = io;

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        indexerTable = inst.getTable("Indexer");
        chuteDetectedPublisher   = indexerTable.getBooleanTopic("Chute/IsFuelDetected").publish();
        chuteDistancePublisher   = indexerTable.getDoubleTopic("Chute/DistanceMeters").publish();
        fuelStatusColorPublisher = indexerTable.getStringTopic("Chute/FuelStatus").publish();
        chuteEmptyPublisher      = indexerTable.getBooleanTopic("Chute/IsChuteEmpty").publish();
    }

    // =====================================================================
    // Periodic
    // =====================================================================
    @Override
    public void periodic() {
        io.updateInputs(inputs);

        double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

        // Fuel detected when the beam distance is shorter than the ball threshold
        isFuelDetected = inputs.chuteDistanceMeters < Constants.Indexer.FUEL_DETECTION_DISTANCE;

        // --- Track first-ever detection and last detection time ---
        if (isFuelDetected) {
            hasSeenFuel = true;
            lastDetectionTimestamp = now;
        }

        if (lastDetectionTimestamp >= 0) {
            secondsSinceLastDetection = now - lastDetectionTimestamp;
        } else {
            secondsSinceLastDetection = Double.POSITIVE_INFINITY;
        }

        publishTelemetry();
    }

    private void publishTelemetry() {
        // TODO: Consider removing these debug publishers once chute tuning is finished.
        chuteDetectedPublisher.set(isFuelDetected);
        chuteDistancePublisher.set(inputs.chuteDistanceMeters);
        // YELLOW = fuel present, RED = no fuel (matches Elastic Dashboard color widget)
        fuelStatusColorPublisher.set(isFuelDetected ? "YELLOW" : "RED");
        chuteEmptyPublisher.set(isChuteEmpty());
    }

    // =====================================================================
    // State
    // =====================================================================
    private void setState(IndexerState state) {
        this.currentState = state;
    }

    public IndexerState getState() {
        return currentState;
    }

    // =====================================================================
    // Motor Control
    // =====================================================================
    public void conveyorForward() {
        io.setConveyorMotor(Constants.Indexer.CONVEYOR_FORWARD_VOLTAGE);
    }

    public void reverseConveyor() {
        io.setConveyorMotor(Constants.Indexer.CONVEYOR_REVERSE_VOLTAGE);
    }

    public void conveyorAirPopper() {
        io.setConveyorMotor(Constants.Indexer.CONVEYOR_POPPER_VOLTAGE);
    }

    public void conveyorStop() {
        io.setConveyorMotor(0.0);
    }

    public void kickerForward() {
        io.setKickerMotorVolts(Constants.Indexer.KICKER_FORWARD_VOLTAGE);
    }

    public void indexerReverse() {
        io.setKickerMotorVolts(Constants.Indexer.KICKER_REVERSE_VOLTAGE);
    }

    public void indexerAirPopper() {
        io.setKickerMotorVolts(Constants.Indexer.KICKER_POPPER_VOLTAGE);
    }

    public void indexerStop() {
        io.setKickerMotorVolts(0.0);
    }

    public void stop() {
        io.stop();
    }

    public void setConveyorVolts(double volts) {
        io.setConveyorMotor(volts);
    }

    public void setIndexerVolts(double volts) {
        io.setKickerMotorVolts(volts);
    }

    // =====================================================================
    // Sensor Queries
    // =====================================================================
    public boolean isFuelDetected() {
        return isFuelDetected;
    }

    public double getSecondsSinceLastDetection() {
        return secondsSinceLastDetection;
    }

    /**
     * True when the chute is clear AND fuel was previously seen.
     * Requires fuel to have been detected at least once (hasSeenFuel guard),
     * then not detected for FUEL_CLEAR_TIME seconds (default 2 s).
     *
     * Use this in auton to know a shot is complete and it is safe to move on.
     */
    public boolean isChuteEmpty() {
        return hasSeenFuel
            && !isFuelDetected
            && secondsSinceLastDetection >= Constants.Indexer.FUEL_CLEAR_TIME;
    }

    /**
     * Resets chute tracking state (hasSeenFuel, lastDetectionTimestamp).
     * Call this before each auton shot so the empty-chute check starts fresh.
     */
    public void resetChuteTracking() {
        hasSeenFuel = false;
        lastDetectionTimestamp = -1.0;
        secondsSinceLastDetection = Double.POSITIVE_INFINITY;
    }

    // =====================================================================
    // Command Factories
    // =====================================================================

    // Single-subsystem commands live here because they are tightly coupled to
    // this subsystem's motors and state. Commands that coordinate multiple
    // subsystems belong in RobotContainer or a superstructure class.

    /**
     * Reverses conveyor and indexer while the button is held.
     * Use to back a ball out of the chute when it has entered prematurely.
     * Stops and returns to IDLE on release.
     */
    public Command reverse() {
        return Commands.startEnd(
                () -> {
                    setState(IndexerState.EJECTING);
                    // conveyorReverse();
                    indexerReverse();
                },
                () -> {
                    stop();
                    setState(IndexerState.IDLE);
                },
                this).withName("Reverse Indexer");
    }

    /**
     * Runs conveyor and indexer forward while the button is held.
     * Stops and returns to IDLE on release.
     */
    public Command feed() {
        return Commands.startEnd(
                () -> {
                    setState(IndexerState.FEEDING);
                    conveyorForward();
                    kickerForward();
                },
                () -> {
                    stop();
                    setState(IndexerState.IDLE);
                },
                this).withName("Feed Fuel");
    }

    /**
     * Runs conveyor and indexer forward for a fixed duration, then stops.
     * Useful in autonomous sequences.
     *
     * @param durationSeconds How long to run the motors
     */
    public Command feedTimed(double durationSeconds) {
        return Commands.sequence(
                Commands.runOnce(() -> {
                    setState(IndexerState.FEEDING);
                    conveyorForward();
                    kickerForward();
                }, this),
                Commands.waitSeconds(durationSeconds),
                Commands.runOnce(() -> {
                    stop();
                    setState(IndexerState.IDLE);
                }, this)).withName("Feed Fuel Timed(" + durationSeconds + "s)");
    }

    /**
     * Reverses both motors for a fixed duration to clear a jam, then stops.
     *
     * @param durationSeconds How long to run in reverse
     */
    public Command eject(double durationSeconds) {
        return Commands.sequence(
                Commands.runOnce(() -> {
                    setState(IndexerState.EJECTING);
                    reverseConveyor();
                    indexerReverse();
                }, this),
                Commands.waitSeconds(durationSeconds),
                Commands.runOnce(() -> {
                    stop();
                    setState(IndexerState.IDLE);
                }, this)).withName("Eject(" + durationSeconds + "s)");
    }

    /** Stops all motors immediately and returns to IDLE. */
    public Command stopCommand() {
        return Commands.runOnce(() -> {
            stop();
            setState(IndexerState.IDLE);
        }, this).withName("StopIndexer");
    }

    /**
     * Resets chute tracking then waits until {@link #isChuteEmpty()} returns true.
     *
     * Intended for auton sequences: call this after starting a shot to block until
     * the ball has fully cleared the chute (no detection for FUEL_CLEAR_TIME seconds).
     *
     * <pre>
     * Commands.sequence(
     *     shooter.spinUp(),
     *     indexer.feedAndWaitForEmpty(),   // feeds AND waits until chute clears
     *     drivetrain.driveToPose(nextPose)
     * )
     * </pre>
     *
     * Pair with a timeout so a missed ball doesn't stall auton forever:
     * {@code indexer.waitForChuteEmpty().withTimeout(3.0)}
     */
    public Command waitForChuteEmpty() {
        return Commands.sequence(
            Commands.runOnce(this::resetChuteTracking),
            Commands.waitUntil(this::isChuteEmpty)
        ).withName("WaitForChuteEmpty");
    }

    /**
     * Feeds conveyor + indexer until the chute sensor confirms the ball has cleared,
     * with a hard-stop safety timeout in case the sensor misses it.
     *
     * <p>Won't exit early — {@link #isChuteEmpty()} requires {@code hasSeenFuel} to
     * be true first, so the feed cannot stop until a ball has been detected at least
     * once. Tracking is reset on start so state from a prior shot doesn't carry over.
     *
     * <p>Replace {@code indexer.feed().withTimeout(feedSeconds)} in auton shoot
     * commands with this to get sensor-based completion with a timer fallback.
     *
     * @param safetyTimeout Hard stop in seconds — prevents auton from hanging if the
     *                      sensor fails or the ball never reaches the chute.
     */
    public Command feedUntilChuteEmpty(double safetyTimeout) {
        return Commands.sequence(
            Commands.runOnce(this::resetChuteTracking),
            Commands.startEnd(
                () -> {
                    setState(IndexerState.FEEDING);
                    conveyorForward();
                    kickerForward();
                },
                () -> {
                    stop();
                    setState(IndexerState.IDLE);
                },
                this
            ).until(this::isChuteEmpty)
             .withTimeout(safetyTimeout)
        ).withName("FeedUntilChuteEmpty");
    }

    /**
     * Ramps conveyor voltage from 0 V to {@code targetVolts} across
     * {@code rampSeconds}, then holds at target until interrupted.
     */
    public Command conveyorRampUp(double rampSeconds, double targetVolts) {
        final double[] startTimeSec = new double[1];

        return Commands.startRun(
                () -> {
                    setState(IndexerState.SENDING_FUEL);
                    startTimeSec[0] = Timer.getFPGATimestamp();
                },
                () -> {
                    double elapsed = Timer.getFPGATimestamp() - startTimeSec[0];
                    double progress = Math.min(1.0, elapsed / Math.max(0.001, rampSeconds));
                    setConveyorVolts(targetVolts * progress);
                },
                this)
                .finallyDo(() -> {
                    conveyorStop();
                    setState(IndexerState.IDLE);
                })
                .withName("ConveyorRampUp");
    }

    /** Test profile: -8 V for 250 ms, then +2 V until interrupted. */
    public Command conveyorReverseThenForwardHold() {
        return Commands.sequence(
                Commands.runOnce(() -> {
                    setState(IndexerState.SENDING_FUEL);
                    setConveyorVolts(-8.0);
                }, this),
                Commands.waitSeconds(0.250),
                Commands.run(() -> setConveyorVolts(2.0), this))
                .finallyDo(() -> {
                    conveyorStop();
                    setState(IndexerState.IDLE);
                })
                .withName("ConveyorReverseThenForwardHold");
    }

    /** Test profile: 4 V for 1 s, then 2 V for 2 s, repeated until interrupted. */
    public Command conveyorPulseProfile() {
        return Commands.repeatingSequence(
                Commands.runOnce(() -> {
                    setState(IndexerState.SENDING_FUEL);
                    setConveyorVolts(4.0);
                }, this),
                Commands.waitSeconds(1.0),
                Commands.runOnce(() -> setConveyorVolts(2.0), this),
                Commands.waitSeconds(2.0))
                .finallyDo(() -> {
                    conveyorStop();
                    setState(IndexerState.IDLE);
                })
                .withName("ConveyorPulseProfile");
    }
}
