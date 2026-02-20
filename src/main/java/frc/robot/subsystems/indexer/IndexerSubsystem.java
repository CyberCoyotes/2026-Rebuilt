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

/**
 * IndexerSubsystem - Moves game pieces from intake through the robot to the shooter.
 *
 * IO Pattern:
 * - IndexerIO: Interface defining what hardware can do
 * - IndexerIOHardware: Real hardware implementation (motors + sensors)
 * - IndexerIOSim: Simulation for testing without hardware
 */
public class IndexerSubsystem extends SubsystemBase {

  // ===== IO Layer =====
  private final IndexerIO io;
  private final IndexerIOInputs inputs = new IndexerIOInputs();

  // ===== NetworkTables Publishers for Elastic Dashboard =====
  private final NetworkTable indexerTable;
  private final StringPublisher statePublisher;
//   private final BooleanPublisher hopperAPublisher;
//   private final BooleanPublisher hopperBPublisher;
  private final DoublePublisher conveyorVelocityPublisher;
  private final DoublePublisher indexerVelocityPublisher;

  
  // ===== Basic volt presets =====
  public static final double CONVEYOR_FORWARD_VOLTAGE = 8.0;   // TODO tune
  public static final double CONVEYOR_REVERSE_VOLTAGE = -8.0;
  public static final double INDEXER_FORWARD_VOLTAGE = 5.0;    // TODO tune
  public static final double INDEXER_REVERSE_VOLTAGE = -5.0;

  // ===== State Tracking =====
  private String currentState = "IDLE";

  public IndexerSubsystem(IndexerIO io) {
    this.io = io;

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    indexerTable = inst.getTable("Indexer");

    statePublisher = indexerTable.getStringTopic("State").publish();
    // hasGamePiecePublisher = indexerTable.getBooleanTopic("HasGamePiece").publish();
    // hopperFullPublisher = indexerTable.getBooleanTopic("HopperFull").publish();
    // hopperCountPublisher = indexerTable.getIntegerTopic("HopperCount").publish();
    // hopperAPublisher = indexerTable.getBooleanTopic("HopperA").publish();
    // hopperBPublisher = indexerTable.getBooleanTopic("HopperB").publish();
    conveyorVelocityPublisher = indexerTable.getDoubleTopic("ConveyorVelocityRPS").publish();
    indexerVelocityPublisher = indexerTable.getDoubleTopic("IndexerVelocityRPS").publish();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    publishTelemetry();
  }

  private void publishTelemetry() {
    statePublisher.set(currentState);
    // hopperAPublisher.set(inputs.hopperADetected);
    // hopperBPublisher.set(inputs.hopperBDetected);
    conveyorVelocityPublisher.set(inputs.conveyorVelocityRPS);
    indexerVelocityPublisher.set(inputs.indexerVelocityRPS);

  }

  // ===== State =====
  public void setState(String state) {
    this.currentState = state;
  }

  // ===== Motor Control =====
  public void setConveyorMotorVolts(double volts) {
    io.setConveyorMotor(volts);
  }

  public void conveyorForward() {
    io.setConveyorMotor(CONVEYOR_FORWARD_VOLTAGE);
  }

  public void conveyorReverse() {
    io.setConveyorMotor(CONVEYOR_REVERSE_VOLTAGE);
  }

  public void conveyorStop() {
    io.setConveyorMotor(0.0);
  }

  public void setIndexerMotorVolts(double volts) {
    io.setIndexerMotor(volts);
  }

  public void indexerForward() {
    io.setIndexerMotor(INDEXER_FORWARD_VOLTAGE);
  }

  public void indexerReverse() {
    io.setIndexerMotor(INDEXER_REVERSE_VOLTAGE);
  }

  public void indexerStop() {
    io.setIndexerMotor(0.0);
  }

  /** Stops both motors immediately. */
  public void stop() {
    io.stop();
  }

  // ===== Simple Command Factories (moved from IndexerCommands) =====

  /**
   * Runs conveyor + indexer forward continuously until interrupted.
   * Designed for use inside SuperstructureCommands with whileTrue() —
   * stops automatically when the parent command is cancelled.
   */
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
    ).withName("Indexer.feed");
  }

  /** Runs conveyor+indexer forward for a duration, then stops and returns to IDLE. */
  public Command feedTimed(double durationSeconds) {
    return Commands.sequence(
            Commands.runOnce(() -> setState("FEED_TIMED"), this),
            Commands.runOnce(() -> {
              conveyorForward();
              indexerForward();
            }, this),
            Commands.waitSeconds(durationSeconds),
            stopCommand())
        .withName("Indexer.feedTimed(" + durationSeconds + "s)");
  }

  /** Runs conveyor+indexer reverse for a duration, then stops and returns to IDLE. */
  public Command ejectTimed(double durationSeconds) {
    return Commands.sequence(
            Commands.runOnce(() -> setState("EJECT"), this),
            Commands.runOnce(() -> {
              conveyorReverse();
              indexerReverse();
            }, this),
            Commands.waitSeconds(durationSeconds),
            stopCommand())
        .withName("Indexer.ejectTimed(" + durationSeconds + "s)");
  }

  /** Stops motors and sets state to IDLE. */
  public Command stopCommand() {
    return Commands.runOnce(() -> {
          stop();
          setState("IDLE");
        }, this)
        .withName("Indexer.stop");
  }

  // ===== Sensors =====


  public boolean isGamePieceInHopper() {
    return inputs.hopperADetected || inputs.hopperBDetected || inputs.hopperCDetected;
  }

  // ===== Telemetry Getters =====
  public double getConveyorVelocityRPS() {
    return inputs.conveyorVelocityRPS;
  }

  public double getIndexerVelocityRPS() {
    return inputs.indexerVelocityRPS;
  }

  public double getConveyorCurrentAmps() {
    return inputs.conveyorCurrentAmps;
  }

  public double getIndexerCurrentAmps() {
    return inputs.indexerCurrentAmps;
  }

  public double getIndexerTofDistanceMM() {
    return inputs.tofDistanceMM;
  }

}
