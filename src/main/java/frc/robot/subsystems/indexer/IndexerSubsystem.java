package frc.robot.subsystems.indexer;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.indexer.IndexerIO.IndexerIOInputs;

public class IndexerSubsystem extends SubsystemBase {

    // ── IO Layer ───────────────────────────────────────────────────────────────
    private final IndexerIO io;
    private final IndexerIOInputs inputs = new IndexerIOInputs();

    // ── Constants ──────────────────────────────────────────────────────────────
    // TODO: Reconcile these with values previously in IndexerCommands
    //       (FEED_VOLTS=9.6, CONVEYOR_VOLTS=7.2) — verify on robot which are correct
    public static final double CONVEYOR_FORWARD_VOLTAGE = 4.0;  // TODO: Tune
    public static final double CONVEYOR_REVERSE_VOLTAGE = -4.0;
    public static final double INDEXER_FORWARD_VOLTAGE  = 4.0;  // TODO: Tune
    public static final double INDEXER_REVERSE_VOLTAGE  = -4.0;

    // Jam detection — jammed when current is HIGH and velocity is LOW simultaneously
    // public static final double HOPPER_JAM_CURRENT_THRESHOLD   = 20.0; // amps — TODO: Tune
    // public static final double HOPPER_JAM_VELOCITY_THRESHOLD  = 0.5;  // RPS  — TODO: Tune
    // public static final double INDEXER_JAM_CURRENT_THRESHOLD  = 20.0; // amps — TODO: Tune
    // public static final double INDEXER_JAM_VELOCITY_THRESHOLD = 0.5;  // RPS  — TODO: Tune

    // ── Dashboard Publishers ───────────────────────────────────────────────────
    private final NetworkTable indexerTable;
    private final StringPublisher statePublisher;
    private final BooleanPublisher hasGamePiecePublisher;
    private final BooleanPublisher hopperFullPublisher;
    private final IntegerPublisher hopperCountPublisher;
    private final BooleanPublisher hopperAPublisher;
    private final BooleanPublisher hopperBPublisher;
    private final DoublePublisher conveyorVelocityPublisher;
    private final DoublePublisher indexerVelocityPublisher;
    // private final BooleanPublisher hopperJammedPublisher;
    // private final BooleanPublisher indexerJammedPublisher;

    // ── State ──────────────────────────────────────────────────────────────────
    private String currentState = "IDLE";

    public IndexerSubsystem(IndexerIO io) {
        this.io = io;

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        indexerTable = inst.getTable("Indexer");

        statePublisher            = indexerTable.getStringTopic("State").publish();
        hasGamePiecePublisher     = indexerTable.getBooleanTopic("HasGamePiece").publish();
        hopperFullPublisher       = indexerTable.getBooleanTopic("HopperFull").publish();
        hopperCountPublisher      = indexerTable.getIntegerTopic("HopperCount").publish();
        hopperAPublisher          = indexerTable.getBooleanTopic("HopperA").publish();
        hopperBPublisher          = indexerTable.getBooleanTopic("HopperB").publish();
        conveyorVelocityPublisher = indexerTable.getDoubleTopic("ConveyorVelocityRPS").publish();
        indexerVelocityPublisher  = indexerTable.getDoubleTopic("IndexerVelocityRPS").publish();
        // hopperJammedPublisher     = indexerTable.getBooleanTopic("HopperJammed").publish();
        // indexerJammedPublisher    = indexerTable.getBooleanTopic("IndexerJammed").publish();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        // Logger.processInputs("Indexer", inputs); // FIXME: enable when AdvantageKit wired up
        publishTelemetry();
    }

    private void publishTelemetry() {
        statePublisher.set(currentState);
        hasGamePiecePublisher.set(isGamePieceAtIndexer());
        hopperFullPublisher.set(isHopperFull());
        hopperCountPublisher.set(getHopperGamePieceCount());
        hopperAPublisher.set(inputs.hopperADetected);
        hopperBPublisher.set(inputs.hopperBDetected);
        conveyorVelocityPublisher.set(inputs.conveyorVelocityRPS);
        indexerVelocityPublisher.set(inputs.indexerVelocityRPS);
        // hopperJammedPublisher.set(isHopperJammed());
        // indexerJammedPublisher.set(isIndexerJammed());
    }

    // ── State ──────────────────────────────────────────────────────────────────
    public void setState(String state) {
        this.currentState = state;
    }

    // ── Motor Control ──────────────────────────────────────────────────────────
    public void conveyorForward()                    { io.setConveyorMotor(CONVEYOR_FORWARD_VOLTAGE); }
    public void conveyorReverse()                    { io.setConveyorMotor(CONVEYOR_REVERSE_VOLTAGE); }
    public void conveyorStop()                       { io.setConveyorMotor(0.0); }
    public void setConveyorVolts(double volts)       { io.setConveyorMotor(volts); }

    public void indexerForward()                     { io.setIndexerMotor(INDEXER_FORWARD_VOLTAGE); }
    public void indexerReverse()                     { io.setIndexerMotor(INDEXER_REVERSE_VOLTAGE); }
    public void indexerStop()                        { io.setIndexerMotor(0.0); }
    public void setIndexerVolts(double volts)        { io.setIndexerMotor(volts); }

    public void stop()                               { io.stop(); }

    // ── Sensor Queries ─────────────────────────────────────────────────────────
    public boolean isGamePieceAtIndexer() { return inputs.gamePieceDetected; }
    public boolean isGamePieceAtHopperA() { return inputs.hopperADetected; }
    public boolean isGamePieceAtHopperB() { return inputs.hopperBDetected; }
    public boolean isGamePieceAtHopperC() { return inputs.hopperCDetected; }

    public boolean isGamePieceInHopper() {
        return inputs.hopperADetected || inputs.hopperBDetected || inputs.hopperCDetected;
    }

    public boolean isHopperFull() {
        return inputs.hopperADetected && inputs.hopperBDetected && inputs.hopperCDetected;
    }

    public int getHopperGamePieceCount() {
        int count = 0;
        if (inputs.hopperADetected) count++;
        if (inputs.hopperBDetected) count++;
        if (inputs.hopperCDetected) count++;
        return count;
    }

    // ── Jam Detection ──────────────────────────────────────────────────────────
    // public boolean isHopperJammed() {
    //     boolean highCurrent = inputs.conveyorCurrentAmps >= HOPPER_JAM_CURRENT_THRESHOLD;
    //     boolean lowVelocity = Math.abs(inputs.conveyorVelocityRPS) <= HOPPER_JAM_VELOCITY_THRESHOLD;
    //     return highCurrent && lowVelocity;
    // }

    // public boolean isIndexerJammed() {
    //     boolean highCurrent = inputs.indexerCurrentAmps >= INDEXER_JAM_CURRENT_THRESHOLD;
    //     boolean lowVelocity = Math.abs(inputs.indexerVelocityRPS) <= INDEXER_JAM_VELOCITY_THRESHOLD;
    //     return highCurrent && lowVelocity;
    // }

    // ── Commands ───────────────────────────────────────────────────────────────

    /** Runs conveyor and indexer forward for a set duration, then stops. */
    public Command feedTimed(double durationSeconds) {
        return Commands.sequence(
            Commands.runOnce(() -> {
                setState("FEEDING");
                conveyorForward();
                indexerForward();
            }, this),
            Commands.waitSeconds(durationSeconds),
            Commands.runOnce(() -> {
                stop();
                setState("IDLE");
            }, this)
        ).withName("FeedTimed");
    }

    /** Runs conveyor and indexer forward while held, stops on release. */
    public Command feed() {
        return Commands.startEnd(
            () -> {
                setState("FEEDING");
                conveyorForward();
                indexerForward();
            },
            () -> {
                stop();
                setState("IDLE");
            },
            this
        ).withName("Feed");
    }

    /** Reverses both motors for a set duration to clear a jam, then stops. */
    public Command eject(double durationSeconds) {
        return Commands.sequence(
            Commands.runOnce(() -> {
                setState("EJECTING");
                conveyorReverse();
                indexerReverse();
            }, this),
            Commands.waitSeconds(durationSeconds),
            Commands.runOnce(() -> {
                stop();
                setState("IDLE");
            }, this)
        ).withName("Eject");
    }

    /** Stops all motors immediately. */
    public Command stopCommand() {
        return Commands.runOnce(() -> {
            stop();
            setState("IDLE");
        }, this).withName("StopIndexer");
    }
}