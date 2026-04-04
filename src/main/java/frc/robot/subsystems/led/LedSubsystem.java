package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.LarsonBounceValue;
import com.ctre.phoenix6.signals.RGBWColor;

import java.util.function.DoubleSupplier;

/**
 * LED subsystem for FRC robot.
 *
 * Slot 0 = left fire, Slot 1 = right fire, other animations override both slots
 */
public class LedSubsystem extends SubsystemBase {

    private final CANdle m_candle = new CANdle(Constants.Led.CANDLE_ID, kCANBus);

    // FIXME There should NOT be a direct dependency here!
    // // Joel feel free to refactor this out however you want,
    // private final ShooterSubsystem m_shooter;

    private final int kLeftFireSlot = 0;
    private final int kRightFireSlot = 1;
    private final int kAnimSlot = 0; // slot for intake/shooting animations

    private static final int kStripCount = 325;

    private final Timer m_animTimer = new Timer();

    // -------------------------------------------------------------------------
// Slot 4 — Shift Warning (< 10 seconds until next hub shift)
// -------------------------------------------------------------------------
private final StrobeAnimation m_shiftWarningAnimation = new StrobeAnimation(0, LEDendIndex)
    .withSlot(4)
    .withColor(new RGBWColor(255, 255, 255, 255))
    .withBrightness(.8)
    .withFrameRate(Hertz.of(10));

// -------------------------------------------------------------------------
// Slot 5 — Red Hub Active
// -------------------------------------------------------------------------
private final SolidColor m_redHubAnimation = new SolidColor(0, LEDendIndex)
    .withSlot(5)
    .withColor(new RGBWColor(255, 0, 0, 0));

// -------------------------------------------------------------------------
// Slot 6 — Blue Hub Active
// -------------------------------------------------------------------------
private final SolidColor m_blueHubAnimation = new SolidColor(0, LEDendIndex)
    .withSlot(6)
    .withColor(new RGBWColor(0, 0, 255, 0));


    private final ControlRequest[] m_animations = new ControlRequest[] {
    m_defaultAnimation,        // 0
    m_spinningUpAnimation,     // 1
    m_readyAnimation,          // 2
    m_extraAnimation,          // 3
    m_shiftWarningAnimation,   // 4
    m_redHubAnimation,         // 5
    m_blueHubAnimation,        // 6
};


    // Left fire region (0–50)
    private final FireAnimation m_leftFire = new FireAnimation(0, 100)
            .withSlot(kLeftFireSlot)
            .withBrightness(1)
            .withDirection(AnimationDirectionValue.Forward)
            .withSparking(1)
            .withCooling(0.2)
            .withFrameRate(Hertz.of(100));

    private DoubleSupplier m_shiftTimeSupplier = () -> 0.0;


    public LedSubsystem() {
        setDefaultCommand(updateLEDs());
    }

    /** Applies only the currently selected animation each loop. */
public Command updateLEDs() {
    return run(() -> {
        if (m_currentSlot == 4) {
            double t = m_shiftTimeSupplier.getAsDouble();
            double hz;
            if (t > 5.0) {
                // 10s → 5s: slow flash (1 Hz → 4 Hz)
                hz = 1.0 + (10.0 - t) * 0.6;
            } else if (t > 1.0) {
                // 5s → 1s: fast blink (4 Hz → 12 Hz)
                hz = 4.0 + (5.0 - t) * 2.0;
            } else {
                // 1s → 0s: almost solid (12 Hz → 30 Hz)
                hz = 12.0 + (1.0 - t) * 18.0;
            }
            m_shiftWarningAnimation.withFrameRate(Hertz.of(hz));
        }
        m_candle.setControl(m_animations[m_currentSlot]);
    });
}


    // ============================================
    // Commands to change state
    // ============================================
    public Command showIdle() {
        return Commands.runOnce(() -> m_state = LedState.IDLE);
    }

    public Command showIntaking() {
        return Commands.runOnce(() -> m_state = LedState.INTAKING);
    }

    public Command showShooting() {
        return Commands.runOnce(() -> m_state = LedState.SHOOTING);
    }

    /** Red hub is the active scoring target. */
public Command showRedHub() {
    return runOnce(() -> m_currentSlot = 5);
}

/** Blue hub is the active scoring target. */
public Command showBlueHub() {
    return runOnce(() -> m_currentSlot = 6);
}

/** Shift is less than 10 seconds away — strobe warning. */
public Command showShiftWarning() {
    return runOnce(() -> m_currentSlot = 4);
}
/** Return LEDs to the default animation. */
public Command showDefault() {
    return runOnce(() -> m_currentSlot = 0);
}

public void setShiftTimeSupplier(DoubleSupplier supplier) {
    m_shiftTimeSupplier = supplier;
}



    // =========================================================================
    // Manual POV cycling (operator testing)
    // =========================================================================

    /** Step forward through all animation slots (wraps around). */
    public Command cycleNext() {
        return Commands.runOnce(() -> {
            LedState[] states = LedState.values();
            m_state = states[(m_state.ordinal() + 1) % states.length];
        });
    }

    public Command cyclePrev() {
        return Commands.runOnce(() -> {
            LedState[] states = LedState.values();
            m_state = states[(m_state.ordinal() - 1 + states.length) % states.length];
        });
    }

    public LedState getState() {
        return m_state;
    }
}