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
    // Camera mounting - Primarily Documentation Purposes
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
    public static final double CAMERA_ANGLE_DEGREES = 15.5;

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
    public static final Translation2d RED_HUB_LOCATION  = new Translation2d(11.923, 4.025);

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

    // started at 40; now 10;
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

    public static final double LEAD_COMPENSATION_DEG_PER_MPS = 00; // Tune up from 0 — 50 degrees of aim offset per m/s of lateral velocity

  }
