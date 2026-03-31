package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.RGBWColor;

import frc.robot.subsystems.shooter.ShooterSubsystem;

/**
 * Controls the external LED strip connected to the CANdle.
 *
 * Strip: 160 LEDs, indices 0–159, mounted in a rough circle.
 *
 * States:
 *   IDLE      — rainbow circling (CTRE animation, always on unless overridden)
 *   INTAKING  — 5-LED red bar bouncing back and forth, all other LEDs white
 *   SHOOTING  — 5-LED red bar spinning around the circle, speed tied to flywheel RPM
 *
 * Everything is driven from periodic() only — no WPILib command scheduler
 * involvement for LED output, so there are no scheduling conflicts.
 */
public class LedSubsystem extends SubsystemBase {

    // -------------------------------------------------------------------------
    // Hardware
    // -------------------------------------------------------------------------
    private final CANBus   kCANBus  = new CANBus("rio");
    private final CANdle   m_candle = new CANdle(15, kCANBus);

    private static final int kStripStart = 0;
    private static final int kStripCount = 160;
    private static final int kStripEnd   = kStripStart + kStripCount - 1; // 159

    // -------------------------------------------------------------------------
    // Shooter reference (for live RPM)
    // -------------------------------------------------------------------------
    private final ShooterSubsystem m_shooter;

    // -------------------------------------------------------------------------
    // Bar animation config
    // -------------------------------------------------------------------------
    private static final int    kBarWidth          = 5;     // LEDs wide for the red bar

    // Intaking bounce
    private static final double kBounceSecsPerTrip = 1.5;   // seconds for bar to travel full strip one way

    // Shooting spin
    private static final double kMinSpinRPM        = 500.0;  // flywheel RPM at which spin starts looking fast
    private static final double kMaxSpinRPM        = 4000.0; // flywheel RPM at max spin speed
    private static final double kMinSpinSecsPerLap = 3.0;    // slowest lap time (seconds per full circle)
    private static final double kMaxSpinSecsPerLap = 0.4;    // fastest lap time (seconds per full circle)

    // -------------------------------------------------------------------------
    // CTRE rainbow animation (IDLE state)
    // -------------------------------------------------------------------------
    private final RainbowAnimation m_rainbowAnimation = new RainbowAnimation(kStripStart, kStripEnd)
            .withSlot(0)
            .withBrightness(0.8)
            .withDirection(AnimationDirectionValue.Forward)
            .withFrameRate(Hertz.of(24));

    // -------------------------------------------------------------------------
    // State
    // -------------------------------------------------------------------------
    public enum LedState { IDLE, INTAKING, SHOOTING }

    private LedState m_state     = LedState.IDLE;
    private LedState m_lastState = null;

    // Timer drives both bounce and spin animations
    private final Timer m_animTimer = new Timer();

    // =========================================================================
    // Constructor
    // =========================================================================
    public LedSubsystem(ShooterSubsystem shooter) {
        m_shooter = shooter;
        m_animTimer.start();
    }

    // =========================================================================
    // Periodic — ONLY place that touches the CANdle
    // =========================================================================
    @Override
    public void periodic() {
        SmartDashboard.putString("LED State", m_state.name());

    if (m_state != m_lastState) {
        m_animTimer.reset();

        if (m_state == LedState.IDLE) {
            m_candle.setControl(m_rainbowAnimation);
        } else {
            // Explicitly stop the CTRE animation slot so it doesn't
            // keep running underneath the manual SolidColor controls
            m_candle.setControl(new SolidColor(kStripStart, kStripEnd)
                .withColor(new RGBWColor(0, 0, 0, 0)));
        }
    }

        switch (m_state) {
            case IDLE:
                // Rainbow is a CTRE animation — set once on entry, runs itself
                break;

            case INTAKING:
                applyBounce();
                break;

            case SHOOTING:
                applySpin();
                break;
        }

        m_lastState = m_state;
    }

    // -------------------------------------------------------------------------
    // Bounce animation (intaking)
    // A 5-LED red bar travels from index 0 → 155, then back from 155 → 0.
    // All other LEDs are white.
    // -------------------------------------------------------------------------
   private void applyBounce() {
    double t          = m_animTimer.get();
    double cycletime  = kBounceSecsPerTrip * 2.0;
    double posInCycle = (t % cycletime) / kBounceSecsPerTrip;
    double frac       = posInCycle <= 1.0 ? posInCycle : 2.0 - posInCycle;

    int maxStart = kStripCount - kBarWidth;
    int barStart = kStripStart + (int) Math.round(frac * maxStart);
    int barEnd   = barStart + kBarWidth - 1;

    // White before bar
    if (barStart > kStripStart)
        m_candle.setControl(new SolidColor(kStripStart, barStart - 1)
            .withColor(new com.ctre.phoenix6.signals.RGBWColor(255, 255, 255, 0)));
    // Red bar
    m_candle.setControl(new SolidColor(barStart, barEnd)
        .withColor(new com.ctre.phoenix6.signals.RGBWColor(255, 0, 0, 0)));
    // White after bar
    if (barEnd < kStripEnd)
        m_candle.setControl(new SolidColor(barEnd + 1, kStripEnd)
            .withColor(new com.ctre.phoenix6.signals.RGBWColor(255, 255, 255, 0)));
}

    // -------------------------------------------------------------------------
    // Spin animation (shooting)
    // A 5-LED red bar circles around the 160-LED strip continuously.
    // Speed is proportional to flywheel RPM, clamped between min and max lap times.
    // All other LEDs are off (black) so the bar pops against the dark background.
        // -------------------------------------------------------------------------
    private void applySpin() {
        double rpm        = Math.abs(m_shooter.getCurrentVelocityRPM());
        double clampedRPM = Math.max(kMinSpinRPM, Math.min(kMaxSpinRPM, rpm));
        double t01        = (clampedRPM - kMinSpinRPM) / (kMaxSpinRPM - kMinSpinRPM);
        double secsPerLap = kMinSpinSecsPerLap + t01 * (kMaxSpinSecsPerLap - kMinSpinSecsPerLap);

        double t      = m_animTimer.get();
        double frac   = (t % secsPerLap) / secsPerLap;
        int barStart  = kStripStart + ((int) Math.round(frac * kStripCount) % kStripCount);
        int barEnd    = barStart + kBarWidth - 1;

        if (barEnd <= kStripEnd) {
            // Bar fits without wrapping
            if (barStart > kStripStart)
                m_candle.setControl(new SolidColor(kStripStart, barStart - 1)
                    .withColor(new com.ctre.phoenix6.signals.RGBWColor(0, 0, 0, 0)));
            m_candle.setControl(new SolidColor(barStart, barEnd)
                .withColor(new com.ctre.phoenix6.signals.RGBWColor(255, 0, 0, 0)));
            if (barEnd < kStripEnd)
                m_candle.setControl(new SolidColor(barEnd + 1, kStripEnd)
                    .withColor(new com.ctre.phoenix6.signals.RGBWColor(0, 0, 0, 0)));
        } else {
            // Bar wraps around — split into two segments
            int wrapEnd = barEnd - kStripCount;
            m_candle.setControl(new SolidColor(kStripStart, wrapEnd)
                .withColor(new com.ctre.phoenix6.signals.RGBWColor(255, 0, 0, 0)));
            if (wrapEnd + 1 < barStart)
                m_candle.setControl(new SolidColor(wrapEnd + 1, barStart - 1)
                    .withColor(new com.ctre.phoenix6.signals.RGBWColor(0, 0, 0, 0)));
            m_candle.setControl(new SolidColor(barStart, kStripEnd)
                .withColor(new com.ctre.phoenix6.signals.RGBWColor(255, 0, 0, 0)));
        }
    }

    // =========================================================================
    // Named State Commands — just flip m_state, no scheduler requirements
    // =========================================================================

    /** Rainbow idle. */
    public Command showIdle() {
        return Commands.runOnce(() -> m_state = LedState.IDLE);
    }

    /** Bouncing red bar on white — intaking. */
    public Command showIntaking() {
        return Commands.runOnce(() -> m_state = LedState.INTAKING);
    }

    /** Spinning red bar tied to flywheel RPM — shooting. */
    public Command showShooting() {
        return Commands.runOnce(() -> m_state = LedState.SHOOTING);
    }

    // =========================================================================
    // Manual operator cycling (for testing)
    // =========================================================================

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