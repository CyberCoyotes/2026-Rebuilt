package frc.robot;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.FuelCommands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

final class AutoRoutineContext {
    final AutoFactory factory;
    final CommandSwerveDrivetrain drivetrain;
    final IndexerSubsystem indexer;
    final IntakeSubsystem intake;
    final ShooterSubsystem shooter;

    AutoRoutineContext(
            AutoFactory factory,
            CommandSwerveDrivetrain drivetrain,
            IndexerSubsystem indexer,
            IntakeSubsystem intake,
            ShooterSubsystem shooter) {
        this.factory = factory;
        this.drivetrain = drivetrain;
        this.indexer = indexer;
        this.intake = intake;
        this.shooter = shooter;
    }

    Command intakeTimed() {
        return intake.intakeFuelTimer(Constants.Auto.INTAKE_TIMEOUT, Constants.Auto.INTAKE_DELAY);
    }

    Command alignAndShoot() {
        return FuelCommands.Auto.poseAlignAndShoot(
                shooter,
                indexer,
                intake,
                drivetrain,
                Constants.Auto.SHOOT_TIMEOUT);
    }

    Command purgeFuel() {
        return FuelCommands.purgeFuel(intake, indexer).withTimeout(Constants.Auto.PURGE_TIMEOUT);
    }
}
