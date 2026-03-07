package frc.robot;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.geometry.Translation2d;

public final class Constants {
  private Constants() {
  }

  /** CTRE CAN bus name (empty string means "rio") */
  // public static final CANBus RIO_CANBUS = new CANBus("rio");
  public static final CANBus RIO_CANBUS = CANBus.roboRIO("rio"); // native rio bus

  // =========================================================
  // Drive / Swerve These are in TunerConstants.java since they're generated with
  // the Phoenix Tuner app
  // =========================================================
  // CAN ID Allocation:
  // 1-12 Drivetrain (drive motors, steer motors, CANcoders) — TunerConstants.java
  // 14 Pigeon 2 (IMU) — TunerConstants.java
  // 15 CANivore / CANdle (LED controller)
  // 20-39 Other motors (intake, indexer, shooter, climber)
  // 40+ Sensors (ToF, CANrange, etc.)
  // CAN Bus:
  // > canivore (for drivetrain + CANdle)
  // > rio (for other subsystems)

  // =========================================================
  // Intake
  // =========================================================
  public static final class Intake {
    private Intake() {
    }

    /** Intake rotator motor - Kraken X44 with TalonFX controller */
    public static final int INTAKE_ROLLER_MOTOR_ID = 20;

    /** Intake slide motor A - Kraken X44 with TalonFX controller */
    public static final int INTAKE_SLIDE_MOTOR_ID = 21;

    /** Intake slide motor B - Kraken X44 with TalonFX controller (paired with A) */
    // public static final int INTAKE_SLIDE_B__MOTOR_ID = 22; // Not needed

    /** Time of Flight sensor - CANrange, confirms fuel presence */
    public static final int INTAKE_SENSOR_ID = 41;

    // === Constants =====================
    public static final double SLIDE_RETRACTED_POSITION = 0.0;
    public static final double SLIDE_EXTENDED_POSITION = 44.40;
    public static final double SLIDE_MIN_POSITION = 0.0;
    public static final double SLIDE_MAX_POSITION = 44.454;
  }

  // =========================================================
  // Indexer
  // =========================================================
  public static final class Indexer {
    private Indexer() {
    }

    /**
     * Indexer motor - Kraken X44 with TalonFX controller, feeds pieces to shooter
     */
    public static final int INDEXER_MOTOR_ID = 23;

    /**
     * Conveyor motor - Minion with TalonFXS controller, moves pieces along hopper
     */
    public static final int CONVEYOR_MOTOR_ID = 24;

    /**
     * CANrange Time of Flight sensor detects presence of fuel at indexer egress to
     * shooter (optional)
     */
    public static final int CHUTE_TOF_ID = 42;

    //=== Voltage Constants =====================
    public static final double CONVEYOR_FORWARD_VOLTAGE = 6.0;
    public static final double CONVEYOR_REVERSE_VOLTAGE = -4.0;
    public static final double CONVEYOR_POPPER_VOLTAGE = 3.0;

    public static final double INDEXER_FORWARD_VOLTAGE = 6.0;
    public static final double INDEXER_REVERSE_VOLTAGE = -4.0;
    public static final double INDEXER_POPPER_VOLTAGE = 3.0;

    public static final double CHUTE_MAX_DISTANCE = 0.50; // TODO: Tune experimentally
    public static final double CHUTE_MIN_DISTANCE = 0.02; // TODO: Tune experimentally

    // Chute detection threshold: distance below which we consider a piece to be
    // present at the chute.
    public static final double CHUTE_DETECTION_THRESHOLD_METERS = 0.40; //
    public static final double CHUTE_TOLERANCE = 0.05;

    public static final double FUEL_CLEAR_TIME = 2.0;

  }

  // =========================================================
  // Shooter
  // =========================================================
  public static final class Shooter {
    private Shooter() {
    }

    /** Flywheel A motor - Kraken X60 with TalonFX controller (leader) */
    public static final int FLYWHEEL_A_MOTOR_ID = 25;

    /** Flywheel B motor - Kraken X60 with TalonFX controller (follower of A) */
    public static final int FLYWHEEL_B_MOTOR_ID = 26;

    /** Flywheel C motor - Kraken X60 with TalonFX controller (follower of A) */
    public static final int FLYWHEEL_C_MOTOR_ID = 27;

    /** Hood motor - Minion with TalonFXS controller, adjusts shot angle */
    public static final int HOOD_MOTOR_ID = 28;

    // === Constants ===
    // Flywheel
    public static final double MAX_FLYWHEEL_RPM = 6000.0; // Kraken X60 free speed (6380 was Falcon 500 — verify against
                                                          // actual motor)
    public static final double IDLE_RPM = 0;

    /**
     * TODO tune RPMs for flywheel without excessive current draw
     * Add an end of line comment `Tuned` when each is verified
     */
    public static final double POPPER_RPM = 650; // TODO: 800 was just a little too much
    public static final double STANDBY_RPM = 1000; // TODO: Tune Standby RPM
    public static final double CLOSE_RPM = 2700; // TODO: Tune was 2600, 4.42
    public static final double TOWER_RPM = 3200; // TODO: Tune was 3100, 4.42
    public static final double TRENCH_RPM = 3200; // TODO: Tune
    public static final double FAR_RPM = 3800; // TODO: Tune was 4000 + 5.5 worked
    public static final double PASS_RPM = 3200; //

    /**
     * Reverse RPM for jam clearing. Only reached through eject(), which gates on
     * EJECT_MAX_ENTRY_RPM.
     */
    public static final double EJECT_RPM = -1500;
    /**
     * Maximum forward flywheel RPM at which EJECT is safe to enter. Prevents
     * violent reversal.
     */
    public static final double EJECT_MAX_ENTRY_RPM = 500.0;

    public static final double FLYWHEEL_TOLERANCE_PERCENT = 0.05; // Tightened from 0.10 — measured steady-state
                                                                  // variance ±30 RPM at 3300; 3% = ±99 RPM (~3×
                                                                  // variance)

    // --- Hood (Kraken rotational positions) ---
    public static final double MIN_HOOD_POSE_ROT = 0.0; // Mechanical limit, validate in configs limit
    public static final double MAX_HOOD_POSE_ROT = 9.14; // Mechanical limit, validate in configs limit
    public static final double HOOD_POSE_TOLERANCE = 0.25; // TODO Tune tolerance based on testing — consider a tighter
                                                           // tolerance than 0.25 rotations

    /**
     * TODO tune Hood rotation position values from Kraken encoder for each shot
     * Consider using WCP Encoder
     * Add an end of line comment `Tuned` when each is verified
     */
    public static final double CLOSE_HOOD = 0.00;   // Tuned and ready
    public static final double POPPER_HOOD = 8.42;  // TODO: Tune Popper hood was 8.42
    public static final double TOWER_HOOD = 4.30;   // TODO: Tune Tower hood
    public static final double TRENCH_HOOD = 4.30;  // TODO: Tune Trench hood
    public static final double FAR_HOOD = 5.50;     // TODO: Tune Far hood, was 4000 + 5.5 worked
    public static final double PASS_HOOD = 3.00;    //TODO: Tune Pass hoodm, was 7.00 and too much

    // --- Testing Increments ---
    public static final double HOOD_TEST_INCREMENT = 0.2;
    public static final double FLYWHEEL_TEST_INCREMENT_RPM = 100.0;
  }

  // =========================================================
  // Climber
  // =========================================================
  public static final class Climber {
    private Climber() {
    }

    /** Climber motor A - Kraken X60 with TalonFX controller */
    public static final int CLIMB_MOTOR_ID = 30;
  }

  // =========================================================
  // Vision / Limelight
  // =========================================================
  public static final class Vision {
    private Vision() {
    }

    // Camera configuration
    public static final String LIMELIGHT3_NAME = "limelight-three";
    public static final String LIMELIGHT4_NAME = "limelight-four";

    // Pipeline indices
    public static final int APRILTAG_PIPELINE = 0;
    public static final int GAME_PIECE_PIPELINE = 1; // Optional: for note/game piece detection

    // =========================================================
    // Camera mounting
    // =========================================================

    /** Height of Limelight lens from floor in meters */
    // 19.25 inches = 0.489 meters
    public static final double CAMERA_HEIGHT_METERS = 0.489;

    // Camera is on the back of robot from center reference
    // -9.5 inches = 0.2413 meters
    public static final double CAMERA_BACK_OFFSET_METERS = 0.2413;

    // Camera is mounted left of center
    // -10.0 inches = 0.0762 meters
    public static final double CAMERA_LEFT_OFFSET_METERS = 0.0762;

    /** Angle of camera from horizontal in degrees (positive = tilted up) */
    // 25 degrees is a common starting point for angled vision setups, but should be
    // measured for accuracy.
    public static final double CAMERA_ANGLE_DEGREES = 25.0; // TODO: Measure actual angle

    /** Height of AprilTag center from floor in meters */
    //
    public static final double APRILTAG_HEIGHT_METERS = 1.45; // TODO: Update for 2026 game

    // Alignment tolerances
    /** Tolerance for horizontal alignment in degrees */
    public static final double ALIGNMENT_TOLERANCE_DEGREES = 1.0; // was 2

    /** Minimum target area to consider target valid (prevents false positives) */
    public static final double MIN_TARGET_AREA_PERCENT = 0.1;

    /** Maximum distance to trust vision measurement in meters */
    public static final double MAX_DISTANCE_METERS = 8.0;

    // Valid tag IDs
    // NOTE: MIN/MAX here are used for general target validation in VisionSubsystem.
    // Hub-specific filtering uses BLUE_HUB_TAG_IDS / RED_HUB_TAG_IDS arrays below.
    public static final int MIN_VALID_TAG_ID = 1;
    public static final int MAX_VALID_TAG_ID = 28; // Fixed: was -1, which rejected all tags

    // Blue hub AprilTag IDs (2026 field layout — all 8 hub faces)
    // Layout: 18/27 (top chute), 19/20 (sides), 26/25 (sides), 21/24 (bottom chute)
    public static final int[] BLUE_HUB_TAG_IDS = {18, 19, 20, 21, 24, 25, 26, 27};

    // Red hub AprilTag IDs (2026 field layout — all 8 hub faces)
    // Layout: 8/5 (top chute), 9/10 (sides), 4/3 (sides), 11/2 (bottom chute)
    public static final int[] RED_HUB_TAG_IDS = {2, 3, 4, 5, 8, 9, 10, 11};

    // Hub center positions in WPILib blue-origin field coordinates (meters).
    // Used by poseAlignAndShoot / autoAlignAndShoot for odometry-based aiming.
    // Red hub is the field-length mirror of blue: x = 17.548 - 4.625 = 12.923
    // TODO: verify exact coordinates against 2026 field layout JSON if shooting accuracy needs improvement
    public static final Translation2d BLUE_HUB_LOCATION = new Translation2d(4.625, 4.025);
    public static final Translation2d RED_HUB_LOCATION  = new Translation2d(12.923, 4.025);

    // State tracking
    /** Time in seconds before considering target "lost" after losing sight */
    public static final double TARGET_TIMEOUT_SECONDS = 0.5;

    // == Vision-driven drivetrain rotation ====
    /**
     * Proportional gain for rotational alignment: (rad/s output) per (degree of tx
     * error).
     *
     * Tuning starting point: 0.06
     * - Too low → slow to center, may not reach ALIGNED before timeout
     * - Too high → oscillates left/right around the target
     * See TUNING.md §5 for step-by-step procedure.
     */
    public static final double ROTATIONAL_KP = 0.10;

    /**
     * Maximum rotational rate the vision command will apply to the drivetrain
     * (rad/s).
     * Prevents violent snap when tx error is large on first acquisition.
     * Default: 3.0 rad/s (~172°/s). Reduce if the robot swings too aggressively.
     */
    public static final double MAX_ALIGNMENT_ROTATION_RAD_PER_SEC = 5.0;

    public static final double ALIGNMENT_TOLERANCE_DEG = 0.5;
    public static final double MAX_ROT_RAD_PER_SEC = 3.0;
    public static final double MIN_DISTANCE_M = 0.5;
    public static final double MAX_DISTANCE_M = 8.0;

    public static final double LEAD_COMPENSATION_DEG_PER_MPS = 40; // Tune up from 0 — 50 degrees of aim offset per m/s of lateral velocity

  }

  // =========================================================
  // LEDs
  // =========================================================
  public static final class Led {
    private Led() {
    }

    /** CANdle device ID */
    public static final int CANDLE_ID = 15;

    /** WS2811 logical segments per meter */
    public static final int SEGMENTS_PER_METER = 20;

    /** WS2811 physical LEDs per logical segment */
    public static final int LEDS_PER_SEGMENT = 36;

    /** Total strip length in meters */
    public static final int STRIP_LENGTH_METERS = 1;

    /** Total number of logical addressable segments */
    public static final int LOGICAL_LED_COUNT = SEGMENTS_PER_METER * STRIP_LENGTH_METERS; // TODO: Adjust as needed
  }
}