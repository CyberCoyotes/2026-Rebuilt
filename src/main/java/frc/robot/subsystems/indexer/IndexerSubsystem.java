package frc.robot.subsystems.indexer;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
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
  private final DoublePublisher conveyorVelocityPublisher;
  private final DoublePublisher indexerVelocityPublisher;

  // ===== Voltage Presets =====
  public static final double CONVEYOR_FORWARD_VOLTAGE = 4.0;   // TODO tune
  public static final double CONVEYOR_REVERSE_VOLTAGE = -4.0;
  public static final double INDEXER_FORWARD_VOLTAGE = 5.0;    // TODO tune
  public static final double INDEXER_REVERSE_VOLTAGE = -5.0;

  public IndexerSubsystem(IndexerIO io) {
    this.io = io;

    NetworkTable indexerTable = NetworkTableInstance.getDefault().getTable("Indexer");
    conveyorVelocityPublisher = indexerTable.getDoubleTopic("ConveyorVelocityRPS").publish();
    indexerVelocityPublisher  = indexerTable.getDoubleTopic("IndexerVelocityRPS").publish();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    conveyorVelocityPublisher.set(inputs.conveyorVelocityRPS);
    indexerVelocityPublisher.set(inputs.indexerVelocityRPS);
  }

  // ===== Motor Control =====

  public void setConveyorMotorVolts(double volts) { io.setConveyorMotor(volts); }

  public void conveyorForward(){
    io.setConveyorMotor(CONVEYOR_FORWARD_VOLTAGE);
  }

  public void conveyorReverse(){
    io.setConveyorMotor(CONVEYOR_REVERSE_VOLTAGE);
  }

  public void conveyorStop(){
    io.setConveyorMotor(0.0);
  }

  public void setIndexerMotorVolts(double volts){
    io.setIndexerMotor(volts);
  }

  public void indexerForward(){
    io.setIndexerMotor(INDEXER_FORWARD_VOLTAGE);
  }

  public void indexerReverse(){
    io.setIndexerMotor(INDEXER_REVERSE_VOLTAGE);
  }

  public void indexerStop(){
    io.setIndexerMotor(0.0);
  }

  /** Stops both motors immediately. */
  public void stop(){
    io.stop();
  }

  // ===== Command Factories =====

  /**
   * Runs conveyor + indexer forward continuously until interrupted.
   * Use with whileTrue() — stops automatically on button release.
   */
  public Command feed() {
    return Commands.startEnd(
        () -> { conveyorForward(); indexerForward(); },
        this::stop,
        this
    ).withName("Indexer.feed");
  }

  /** Runs conveyor + indexer forward for a set duration, then stops. */
  public Command feedTimed(double durationSeconds) {
    return Commands.startEnd(
            () -> { conveyorForward(); indexerForward(); },
            this::stop,
            this)
        .withTimeout(durationSeconds)
        .withName("Indexer.feedTimed(" + durationSeconds + "s)");
  }

  /** Runs conveyor + indexer in reverse for a set duration, then stops. */
  public Command ejectTimed(double durationSeconds) {
    return Commands.startEnd(
            () -> { conveyorReverse(); indexerReverse(); },
            this::stop,
            this)
        .withTimeout(durationSeconds)
        .withName("Indexer.ejectTimed(" + durationSeconds + "s)");
  }

  /** Stops both motors immediately. */
  public Command stopCommand() {
    return Commands.runOnce(this::stop, this).withName("Indexer.stop");
  }

  // ===== Sensors =====


  // ===== Telemetry Getters =====

  public double getConveyorVelocityRPS()  { return inputs.conveyorVelocityRPS; }
  public double getIndexerVelocityRPS()   { return inputs.indexerVelocityRPS; }
  public double getConveyorCurrentAmps()  { return inputs.conveyorCurrentAmps; }
  public double getIndexerCurrentAmps()   { return inputs.indexerCurrentAmps; }
}
