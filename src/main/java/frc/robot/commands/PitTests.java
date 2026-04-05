package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterSubsystem;


public class PitTests {

    /* Methods that build commands should return commands; 
    * the caller decides when and how to schedule them. 
    * It also makes the method composable and could chain it into a larger test sequence later wanted. 
    */

    private final double testTime = 2; //seconds
    
    //Run the flywheel at each preset speed for 4 (s)
    public Command testShooterPresets(ShooterSubsystem shooter) {
        Command close = Commands.run(() -> shooter.setTargetVelocity(Constants.Flywheel.CLOSE_RPM), shooter); 
        Command trench = Commands.run(() -> shooter.setTargetVelocity(Constants.Flywheel.TRENCH_RPM), shooter);
        Command pass = Commands.run(() -> shooter.setTargetVelocity(Constants.Flywheel.PASS_RPM), shooter);
        Command far = Commands.run(() -> shooter.setTargetVelocity(Constants.Flywheel.FAR_RPM), shooter);

        // Ideally it should have something like a `isReady` type check, otherwise it just runs until timeout
        return Commands.sequence(
            close.withTimeout(testTime),
            trench.withTimeout(testTime),
            pass.withTimeout(testTime),
            far.withTimeout(testTime)
        );

        // Popper.withTimeout(4).andThen(Standby.withTimeout(4)).andThen(Close.withTimeout(4)).andThen(Tower.withTimeout(4)).andThen(Trench.withTimeout(4)).andThen(Far.withTimeout(4)).andThen(Pass.withTimeout(4)).schedule();
    }

    //Wait 6 (s)
    //Run the flywheel in reverse for 4 (s)
    public void testShooterEject(ShooterSubsystem shooter) {
        Command Eject = Commands.run(() -> shooter.setTargetVelocity(Constants.Flywheel.EJECT_RPM), shooter);

        Eject.withTimeout(4);
    }
    
    //Wait 4 (s)
    //test hood presets
    public void testHoodPresets(ShooterSubsystem shooter) {
        Command Close = Commands.run(() -> shooter.setTargetVelocity(Constants.Hood.CLOSE_HOOD), shooter);
        Command Tower = Commands.run(() -> shooter.setTargetVelocity(Constants.Hood.TOWER_HOOD), shooter); 
        Command Trench = Commands.run(() -> shooter.setTargetVelocity(Constants.Hood.TRENCH_HOOD), shooter);
        Command Far = Commands.run(() -> shooter.setTargetVelocity(Constants.Hood.FAR_HOOD), shooter);
        Command Pass = Commands.run(() -> shooter.setTargetVelocity(Constants.Hood.PASS_HOOD), shooter);
        Commands.sequence(
            Close.withTimeout(testTime),
            Tower.withTimeout(testTime),
            Trench.withTimeout(testTime),
            Far.withTimeout(testTime),
            Pass.withTimeout(testTime)
        );}

    /*
    public static Command testShooterAndHood(ShooterSubsystem shooter) {
        return Commands.sequence( 
            Commands.runOnce(() -> testShooterForwardPresets(shooter)),
            Commands.waitSeconds(6),
            Commands.runOnce(() -> testShooterEject(shooter)),
            Commands.waitSeconds(4),
            Commands.runOnce(() -> testHoodPresets(shooter))
                );
                */
    }

