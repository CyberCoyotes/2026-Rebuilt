package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.commands.AlignToHubCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class AutoRoutines {
    private final AutoFactory m_factory;
    private final CommandSwerveDrivetrain m_drivetrain;
    private final IntakeSubsystem m_intake;
    private final ShooterSubsystem m_shooter;
    private final IndexerSubsystem m_indexer;
    private final VisionSubsystem m_vision;

    public AutoRoutines(
            AutoFactory factory,
            CommandSwerveDrivetrain drivetrain,
            IntakeSubsystem intake,
            ShooterSubsystem shooter,
            IndexerSubsystem indexer,
            VisionSubsystem vision) {
        m_factory = factory;
        m_drivetrain = drivetrain;
        m_intake = intake;
        m_shooter = shooter;
        m_indexer = indexer;
        m_vision = vision;
    }

    // ===== Shot Constants =====
    private static final double SHOOT_RPM        = 3500.0;
    private static final double SHOOT_HOOD_ANGLE = 4.0;
    private static final double SHOOT_TIMEOUT    = 3.0;

    public AutoRoutine singleCenterShootAuto() {
        final AutoRoutine routine = m_factory.newRoutine("SingleCenterShoot Auto");

        // Segment 0: Start (x:4.25, y:0.65) → mid-field stop (x:7.75, y:3.5)
        final AutoTrajectory segment0 = routine.trajectory("SingleCenterShoot", 0);

        // Segment 1: Mid-field stop (x:7.75, y:3.5) → end/shoot position (x:3.0, y:1.5)
        final AutoTrajectory segment1 = routine.trajectory("SingleCenterShoot", 1);

        routine.active().onTrue(
            // 1. Reset odometry to path start position
            segment0.resetOdometry()

            // 2. Drive first segment — intake runs at 8.0V the entire time
            .andThen(segment0.cmd().alongWith(m_intake.intakeFuel(8.0)))

            // 3. Drive second segment — intake keeps running at 8.0V
            .andThen(segment1.cmd().alongWith(m_intake.intakeFuel(8.0)))

            // 4. Align to hub AprilTag — robot holds still (zero translation) while rotating
            //    Exits once the vision horizontal error is within tolerance
            .andThen(new AlignToHubCommand(
                m_drivetrain,
                m_vision,
                () -> 0.0,
                () -> 0.0
            ).until(this::isAligned))

            // 5. Spin up shooter to set RPM and hood angle
            .andThen(Commands.runOnce(() -> {
                m_shooter.setTargetVelocity(SHOOT_RPM);
                m_shooter.setTargetHoodPose(SHOOT_HOOD_ANGLE);
                m_shooter.prepareToShoot();
            }, m_shooter))

            // 6. Wait until shooter is ready (flywheel at speed + hood at pose)
            .andThen(Commands.waitUntil(m_shooter::isReady))

            // 7. Feed with conveyor + indexer for SHOOT_TIMEOUT seconds, then stop both
            .andThen(Commands.startEnd(
                () -> {
                    m_indexer.setState("FEEDING");
                    m_indexer.conveyorForward();
                    m_indexer.indexerForward();
                },
                () -> {
                    m_indexer.conveyorStop();
                    m_indexer.indexerStop();
                    m_indexer.setState("IDLE");
                    m_shooter.returnToIdle();
                },
                m_indexer
            ).withTimeout(SHOOT_TIMEOUT))
        );

        return routine;
    }

    /**
     * Returns true when the vision horizontal error is within the alignment tolerance.
     * Mirrors the ALIGN_TOLERANCE_DEGREES constant used inside AlignToHubCommand.
     */
    private boolean isAligned() {
        return m_vision.hasTarget()
            && Math.abs(m_vision.getHorizontalAngleDegrees()) <= 2.0;
    }
}