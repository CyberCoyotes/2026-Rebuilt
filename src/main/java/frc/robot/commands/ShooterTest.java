package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShooterTest {
    //Run the flywheel at each preset speed for 4 (s)
    public void testShooterForwardPresets(ShooterSubsystem shooter) {
        Command Popper = Commands.run(() -> shooter.setTargetVelocity(Constants.Shooter.POPPER_RPM), shooter);
        Command Standby = Commands.run(() -> shooter.setTargetVelocity(Constants.Shooter.STANDBY_RPM), shooter);
        Command Close = Commands.run(() -> shooter.setTargetVelocity(Constants.Shooter.CLOSE_RPM), shooter);
        Command Tower = Commands.run(() -> shooter.setTargetVelocity(Constants.Shooter.TOWER_RPM), shooter); 
        Command Trench = Commands.run(() -> shooter.setTargetVelocity(Constants.Shooter.TRENCH_RPM), shooter);
        Command Far = Commands.run(() -> shooter.setTargetVelocity(Constants.Shooter.FAR_RPM), shooter);
        Command Pass = Commands.run(() -> shooter.setTargetVelocity(Constants.Shooter.PASS_RPM), shooter);


        Popper.withTimeout(4).andThen(Standby.withTimeout(4)).andThen(Close.withTimeout(4)).andThen(Tower.withTimeout(4)).andThen(Trench.withTimeout(4)).andThen(Far.withTimeout(4)).andThen(Pass.withTimeout(4)).schedule();
    }
    //Wait 6 (s)
    //Run the flywheel in reverse for 4 (s)
    public void testShooterEject(ShooterSubsystem shooter) {
        Command Eject = Commands.run(() -> shooter.setTargetVelocity(Constants.Shooter.EJECT_RPM), shooter);

        Eject.withTimeout(4).schedule();
    }
    //Wait 4 (s)
    //test hood presets
    public void testHoodPresets(ShooterSubsystem shooter) {
        Command Popper = Commands.run(() -> shooter.setTargetVelocity(Constants.Shooter.POPPER_HOOD), shooter);
        //there is no hood position for standby
        Command Close = Commands.run(() -> shooter.setTargetVelocity(Constants.Shooter.CLOSE_HOOD), shooter);
        Command Tower = Commands.run(() -> shooter.setTargetVelocity(Constants.Shooter.TOWER_HOOD), shooter); 
        Command Trench = Commands.run(() -> shooter.setTargetVelocity(Constants.Shooter.TRENCH_HOOD), shooter);
        Command Far = Commands.run(() -> shooter.setTargetVelocity(Constants.Shooter.FAR_HOOD), shooter);
        Command Pass = Commands.run(() -> shooter.setTargetVelocity(Constants.Shooter.PASS_HOOD), shooter);

        Popper.withTimeout(4).andThen(Close.withTimeout(4)).andThen(Tower.withTimeout(4)).andThen(Trench.withTimeout(4)).andThen(Far.withTimeout(4)).andThen(Pass.withTimeout(4)).schedule();
    }

/*     public static Command testShooterAndHood(ShooterSubsystem shooter) {
        return Commands.sequence( 
            Commands.runOnce(() -> testShooterForwardPresets(shooter)),
            Commands.waitSeconds(6),
            Commands.runOnce(() -> testShooterEject(shooter)),
            Commands.waitSeconds(4),
            Commands.runOnce(() -> testHoodPresets(shooter))
                );
                */
    }

