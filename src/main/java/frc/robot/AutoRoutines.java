package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

import frc.robot.commands.AutoAlignAndShootCommand;
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

    public AutoRoutine singleCenterShootAuto() {
        final AutoRoutine routine = m_factory.newRoutine("SingleCenterShoot Auto");

        final AutoTrajectory segment0 = routine.trajectory("SingleCenterShoot", 0);
        final AutoTrajectory segment1 = routine.trajectory("SingleCenterShoot", 1);

        routine.active().onTrue(
            segment0.resetOdometry()
            .andThen(segment0.cmd().alongWith(m_intake.intakeFuel(8.0)))
            .andThen(segment1.cmd().alongWith(m_intake.intakeFuel(8.0)))
            // deadlineFor() lets AutoAlignAndShootCommand control when this ends.
            // Intake keeps running alongside but does NOT block completion.
            .andThen(new AutoAlignAndShootCommand(
                m_drivetrain,
                m_shooter,
                m_vision,
                m_indexer,
                5.0
            ).deadlineFor(m_intake.intakeFuel(8.0)))
        );

        return routine;
    }
}