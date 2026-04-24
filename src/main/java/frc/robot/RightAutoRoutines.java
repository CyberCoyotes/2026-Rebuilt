package frc.robot;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;

final class RightAutoRoutines {
    private final AutoRoutineContext ctx;

    RightAutoRoutines(AutoRoutineContext ctx) {
        this.ctx = ctx;
    }

    public AutoRoutine RtTrench_Ramp_Single() {
        final AutoRoutine routine = ctx.factory.newRoutine("Right x1 Trench-Ramp");

        final AutoTrajectory rtTrench_Middle = routine.trajectory("RtTrench_MiddleAngry", 0);
        final AutoTrajectory rtRampMiddle_Alliance = routine.trajectory("RtRampMiddle_Alliance", 0);
        final AutoTrajectory rtShootRamp = routine.trajectory("RtShootRamp", 0);

        routine.active().onTrue(
                Commands.sequence(
                        rtTrench_Middle.resetOdometry(),
                        ctx.drivetrain.stop().withTimeout(4.0),
                        Commands.deadline(rtTrench_Middle.cmd(), ctx.intakeTimed()),
                        rtRampMiddle_Alliance.cmd(),
                        rtShootRamp.cmd(),
                        ctx.alignAndShoot()));

        return routine;
    }

    public AutoRoutine RtTrench_Ramp_Meep() {
        final AutoRoutine routine = ctx.factory.newRoutine("Right x1 Trench-Ramp");

        final AutoTrajectory rtTrench_Middle = routine.trajectory("RtTrench_MiddleAngry", 0);
        final AutoTrajectory rtRampMiddle_Alliance = routine.trajectory("RtRampMiddle_Alliance", 0);
        final AutoTrajectory rtShootRamp = routine.trajectory("RtShootRamp", 0);

        routine.active().onTrue(
                Commands.sequence(
                        rtTrench_Middle.resetOdometry(),
                        Commands.deadline(rtTrench_Middle.cmd(), ctx.intakeTimed()),
                        rtRampMiddle_Alliance.cmd(),
                        rtShootRamp.cmd(),
                        ctx.alignAndShoot()));

        return routine;
    }

    public AutoRoutine RtTrench_Ramp_Double() {
        final AutoRoutine routine = ctx.factory.newRoutine("Right x2 Trench-Ramp");
        final AutoTrajectory rtTrench_Middle = routine.trajectory("RtTrench_Middle", 0);
        final AutoTrajectory rtRampMiddle_Alliance = routine.trajectory("RtRampMiddle_Alliance", 0);
        final AutoTrajectory rtShootRamp_Trench = routine.trajectory("RtShootRamp_Trench", 0);
        final AutoTrajectory rtTrench_Middle_2 = routine.trajectory("RtTrench_Middle", 0);
        final AutoTrajectory rtRampMiddle_Alliance_2 = routine.trajectory("RtRampMiddle_Alliance", 0);

        routine.active().onTrue(
                Commands.sequence(
                        rtTrench_Middle.resetOdometry(),
                        Commands.deadline(rtTrench_Middle.cmd(), ctx.intakeTimed()),
                        rtRampMiddle_Alliance.cmd(),
                        ctx.alignAndShoot(),
                        rtShootRamp_Trench.cmd(),
                        Commands.deadline(rtTrench_Middle_2.cmd(), ctx.intakeTimed()),
                        rtRampMiddle_Alliance_2.cmd(),
                        ctx.alignAndShoot()));

        return routine;
    }

    public AutoRoutine RtTrench_Ramp_HubSweep() {
        final AutoRoutine routine = ctx.factory.newRoutine("Right Trench-Ramp-HubSweep");
        final AutoTrajectory rtTrench_Middle = routine.trajectory("RtTrench_Middle", 0);
        final AutoTrajectory rtRampMiddle_Alliance = routine.trajectory("RtRampMiddle_Alliance", 0);
        final AutoTrajectory rtShootRamp_Trench = routine.trajectory("RtShootRamp_Trench", 0);
        final AutoTrajectory rtTrench_HubSweep = routine.trajectory("RtTrench_HubSweep", 0);
        final AutoTrajectory rtHub_Ramp = routine.trajectory("RtHub_Ramp", 0);
        final AutoTrajectory rtRampMiddle_Alliance_2 = routine.trajectory("RtRampMiddle_Alliance", 0);

        routine.active().onTrue(
                Commands.sequence(
                        rtTrench_Middle.resetOdometry(),
                        Commands.deadline(rtTrench_Middle.cmd(), ctx.intakeTimed()),
                        rtRampMiddle_Alliance.cmd(),
                        ctx.alignAndShoot(),
                        rtShootRamp_Trench.cmd(),
                        Commands.deadline(rtTrench_HubSweep.cmd(), ctx.intakeTimed()),
                        rtHub_Ramp.cmd(),
                        rtRampMiddle_Alliance_2.cmd(),
                        ctx.alignAndShoot()));

        return routine;
    }

    public AutoRoutine RtTrench_Ramp_Sweep_Purge() {
        final AutoRoutine routine = ctx.factory.newRoutine("Right Trench-Ramp-Sweep_Purge");
        final AutoTrajectory rtTrench_Middle = routine.trajectory("RtTrench_Middle", 0);
        final AutoTrajectory rtRampMiddle_Alliance = routine.trajectory("RtRampMiddle_Alliance", 0);
        final AutoTrajectory rtShootRamp_Trench = routine.trajectory("RtShootRamp_Trench", 0);
        final AutoTrajectory rtTrench_HubSweep = routine.trajectory("RtTrench_HubSweep", 0);
        final AutoTrajectory rtHub_Purge = routine.trajectory("RtHub_Purge", 0);

        routine.active().onTrue(
                Commands.sequence(
                        rtTrench_Middle.resetOdometry(),
                        Commands.deadline(rtTrench_Middle.cmd(), ctx.intakeTimed()),
                        rtRampMiddle_Alliance.cmd(),
                        ctx.alignAndShoot(),
                        rtShootRamp_Trench.cmd(),
                        Commands.deadline(rtTrench_HubSweep.cmd(), ctx.intakeTimed()),
                        rtHub_Purge.cmd(),
                        ctx.purgeFuel()));

        return routine;
    }

    public AutoRoutine RtTrench_Ramp_Sweep_AngryPurge() {
        return RtTrench_Ramp_Sweep_Purge();
    }

    public AutoRoutine RtTrench_Ramp_AngryMeepMeep() {
        final AutoRoutine routine = ctx.factory.newRoutine("AngryMeepMeep");
        final AutoTrajectory angryMeepMeep = routine.trajectory("RtTrench_AngryMeepMeep", 0);

        routine.active().onTrue(
                Commands.sequence(
                        angryMeepMeep.resetOdometry(),
                        Commands.deadline(angryMeepMeep.cmd(), ctx.intakeTimed()),
                        ctx.alignAndShoot()));

        return routine;
    }

    public AutoRoutine RtBulldozer() {
        final AutoRoutine routine = ctx.factory.newRoutine("RtBulldozer 2026");
        final AutoTrajectory rtBulldozer = routine.trajectory("RtBulldozer2026", 0);
        final AutoTrajectory rtRampMiddle_Alliance = routine.trajectory("RtRampMiddle_Alliance", 0);

        routine.active().onTrue(
                Commands.sequence(
                        rtBulldozer.resetOdometry(),
                        rtBulldozer.cmd(),
                        rtRampMiddle_Alliance.cmd(),
                        ctx.alignAndShoot()));

        rtBulldozer.atTime("Intake").onTrue(ctx.intake.intakeFuelTimer(Constants.Auto.INTAKE_TIMEOUT + 2, 0));
        return routine;
    }
}
