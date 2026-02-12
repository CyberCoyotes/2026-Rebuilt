package frc.robot.subsystems.led;

import com.ctre.phoenix.led.CANdle;
// import com.ctre.phoenix6.hardware.CANdle; // TODO Check for Phoenix 6 release

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem.ShooterState;

/**
 * LED subsystem driven by a CTRE CANdle for WS2811 strip control.
 *
 * The WS2811 COB strip is addressed as logical segments (20 per meter).
 * Colors are applied per segment, not per physical LED.
 */
public class LedSubsystem extends SubsystemBase {
    private static final LedColor IDLE_COLOR = new LedColor(0, 0, 0);
    private static final LedColor SPINUP_COLOR = new LedColor(255, 140, 0); // Orange
    private static final LedColor READY_COLOR = new LedColor(0, 255, 0); // Green
    private static final LedColor PASS_COLOR = new LedColor(0, 0, 255); // Blue
    private static final LedColor EJECT_COLOR = new LedColor(255, 0, 0); // Red

    private final CANdle candle;
    private final ShooterSubsystem shooter;
    private final int ledCount;

    private LedColor lastColor = IDLE_COLOR;

    public LedSubsystem(ShooterSubsystem shooter) {
        this.shooter = shooter;
        this.candle = new CANdle(Constants.Led.CANDLE_ID, Constants.CAN_BUS_NAME);
        this.ledCount = Math.max(1, Constants.Led.LOGICAL_LED_COUNT);
        setSolidColor(IDLE_COLOR);
    }

    @Override
    public void periodic() {
        LedColor desiredColor = getShooterColor();
        if (!desiredColor.equals(lastColor)) {
            setSolidColor(desiredColor);
        }
    }

    private LedColor getShooterColor() {
        ShooterState state = shooter.getState();
        return switch (state) {
            case IDLE -> IDLE_COLOR;
            case SPINUP -> SPINUP_COLOR;
            case READY -> READY_COLOR;
            case PASS -> PASS_COLOR;
            case EJECT -> EJECT_COLOR;
        };
    }

    private void setSolidColor(LedColor color) {
        candle.setLEDs(color.red(), color.green(), color.blue(), 0, 0, ledCount);
        lastColor = color;
    }

    private record LedColor(int red, int green, int blue) {}
}
