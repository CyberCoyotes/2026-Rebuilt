package frc.robot;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.geometry.Translation2d;

public final class Constants {
  private Constants() {}

  /** CTRE CAN bus name (empty string means "rio") */
  public static final CANBus RIO_CANBUS = CANBus.roboRIO("rio");

  // =========================================================
  // Drive / Swerve
  // =========================================================
  // CAN ID Allocation:
  //   1-12  Drivetrain (drive motors, steer motors, CANcoders) — TunerConstants.java
  //   14    Pigeon 2 (IMU) — TunerConstants.java
  //   15    CANivore / CANdle (LED controller)
  //   20-39 Other motors (intake, indexer, shooter, climber)
  //   40+   Sensors (ToF, CANrange, etc.)
  // CAN Bus:
  //  > canivore (for drivetrain + CANdle)
  //  > rio (for other subsystems)

  // =========================================================
  // Intake
  // =========================================================
  public static final class Intake {
    private Intake() {}

    /** Intake rotator motor - Kraken X44 with TalonFX controller */
    public static final int INTAKE_ROLLER_MOTOR_ID = 20;

    /** Intake slide motor A - Kraken X44 with TalonFX controller */
    public static final int INTAKE_SLIDE_MOTOR_ID = 21;

    /** Time of Flight sensor - CANrange, confirms fuel presence */
    public static final int INTAKE_SENSOR_ID = 41;
  }

  // =========================================================
  // Indexer
  // =========================================================
  public static final class Indexer {
    private Indexer() {}

    /** Indexer motor - Kraken X44 with TalonFX controller */
    public static final int INDEXER_MOTOR_ID = 23;

    /** Conveyor motor - Minion with TalonFXS controller */
    public static final int CONVEYOR_MOTOR_ID = 24;

    /** Time of Flight sensor at indexer egress */
    public static final int INDEXER_SENSOR_ID = 42;

    /** Time of Flight sensor A - hopper fullness */
    public static final int HOPPER_A_TOF_ID = 43;

    /** Time of Flight sensor B - hopper fullness */
    public static final int HOPPER_B_TOF_ID = 44;
  }

  // =========================================================
  // Shooter
  // =========================================================
  public static final class Shooter {
    private Shooter() {}

    /** Flywheel A motor - Falcon 500, leader */
    public static final int FLYWHEEL_A_MOTOR_ID = 25;

    /** Flywheel B motor - Falcon 500, follower */
    public static final int FLYWHEEL_B_MOTOR_ID = 26;

    /** Flywheel C motor - Falcon 500, follower */
    public static final int FLYWHEEL_C_MOTOR_ID = 27;

    /** Hood motor - Minion with TalonFXS */
    public static final int HOOD_MOTOR_ID = 28;

    /** WCP ThroughBore Encoder via CANcoder for hood position */
    public static final int HOOD_POSE_ENCODER_ID = 46;
  }

  // =========================================================
  // Climber
  // =========================================================
  public static final class Climber {
    private Climber() {}

    /** Climber motor - Kraken X60 with TalonFX */
    public static final int CLIMB_MOTOR_ID = 30;
  }

  // =========================================================
  // Vision / Limelight
  // =========================================================
  public static final class Vision {
    private Vision() {}

    // -------------------------------------------------------
    // Camera names
    // -------------------------------------------------------
    public static final String LIMELIGHT3_NAME = "limelight-three";
    public static final String LIMELIGHT4_NAME = "limelight-four";

    // -------------------------------------------------------
    // Pipeline indices
    // -------------------------------------------------------
    public static final int APRILTAG_PIPELINE    = 0;
    public static final int GAME_PIECE_PIPELINE  = 1;

    // -------------------------------------------------------
    // Camera mounting geometry (LL4, measured on robot)
    // -------------------------------------------------------
    /** Height of LL4 lens from floor in meters (21" = 0.533m) */
    public static final double CAMERA_HEIGHT_METERS = 0.533;

    /** Upward tilt of LL4 from horizontal in degrees */
    public static final double CAMERA_ANGLE_DEGREES = 10.0;

    /** Height of hub AprilTag center from floor in meters (45" = 1.143m) */
    public static final double APRILTAG_HEIGHT_METERS = 1.143;

    // -------------------------------------------------------
    // Hub field position — WPILib Blue Alliance coordinates
    //
    // The Reefscape 2025 hub center sits at approximately (4.49, 4.11)
    // on the WPILib blue-origin field coordinate system.
    // Verify against your official field layout file if this seems off.
    // -------------------------------------------------------
    // Blue hub center — geometric average of all 8 blue hub AprilTag positions
    // from the official 2026 Rebuilt field layout (FE-2026 Rev A, welded perimeter).
    // Tags 18-21 and 24-27. Calculated from official coordinates: (178.61", 158.84")
    public static final Translation2d HUB_CENTER_BLUE = new Translation2d(4.537, 4.035);

    // -------------------------------------------------------
    // MegaTag2 / pose estimator trust thresholds
    // -------------------------------------------------------
    /** Reject pose estimates where the robot appears further than this from the hub (meters) */
    public static final double MAX_DISTANCE_METERS = 8.0;

    /** Minimum tag count required to trust a MegaTag2 pose estimate */
    public static final int MIN_TAG_COUNT = 1;

    /** Maximum ambiguity ratio to trust a single-tag pose estimate (0 = perfect, 1 = terrible) */
    public static final double MAX_AMBIGUITY = 0.9;

    // -------------------------------------------------------
    // Alignment tolerances
    // -------------------------------------------------------
    /** Horizontal alignment tolerance in degrees for hub targeting */
    public static final double ALIGNMENT_TOLERANCE_DEGREES = 2.0;

    /** Minimum target area to consider a detection valid */
    public static final double MIN_TARGET_AREA_PERCENT = 0.1;

    /** Grace period in seconds before considering a target truly lost */
    public static final double TARGET_TIMEOUT_SECONDS = 0.5;

    // -------------------------------------------------------
    // Valid AprilTag ID range
    // -------------------------------------------------------
    public static final int MIN_VALID_TAG_ID  = 1;
    public static final int MAX_VALID_TAG_ID  = 28;

    // Hub-facing tag IDs (Reefscape 2025 blue alliance hub)
    public static final int BLUE_HUB_MIN_TAG_ID = 18;
    public static final int BLUE_HUB_MAX_TAG_ID = 27;
  }

  // =========================================================
  // LEDs
  // =========================================================
  public static final class Led {
    private Led() {}

    public static final int CANDLE_ID           = 15;
    public static final int SEGMENTS_PER_METER  = 20;
    public static final int LEDS_PER_SEGMENT    = 36;
    public static final int STRIP_LENGTH_METERS = 1;
    public static final int LOGICAL_LED_COUNT   = SEGMENTS_PER_METER * STRIP_LENGTH_METERS;
  }
}