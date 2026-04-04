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
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.LarsonBounceValue;
import com.ctre.phoenix6.signals.RGBWColor;

import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/**
 * LED subsystem for FRC robot.
 *
 * Slot 0 = left fire, Slot 1 = right fire, other animations override both slots
 */
public class LedSubsystem extends SubsystemBase {

    private final CANBus kCANBus = new CANBus("rio");
    private final CANdle m_candle = new CANdle(Constants.Led.CANDLE_ID, kCANBus);

    // FIXME There should NOT be a direct dependency here!
    // Joel feel free to refactor this out however you want,
    private final ShooterSubsystem m_shooter;

    private final int kLeftFireSlot = 0;
    private final int kRightFireSlot = 1;
    private final int kAnimSlot = 0; // slot for intake/shooting animations

    private static final int kStripCount = 325;

    private final Timer m_animTimer = new Timer();

    // ==========================================
    // Animations
    // ==========================================

    // Left fire region (0–50)
    private final FireAnimation m_leftFire = new FireAnimation(0, 100)
            .withSlot(kLeftFireSlot)
            .withBrightness(1)
            .withDirection(AnimationDirectionValue.Forward)
            .withSparking(1)
            .withCooling(0.2)
            .withFrameRate(Hertz.of(100));

    // Right fire region (125–175)
    private final FireAnimation m_rightFire = new FireAnimation(325, 225)
            .withSlot(kRightFireSlot)
            .withBrightness(1)
            .withDirection(AnimationDirectionValue.Forward)
            .withSparking(1)
            .withCooling(0.2)
            .withFrameRate(Hertz.of(100));

    // Intake: Larson animation (overrides both fire slots)
    private final LarsonAnimation m_intakeAnimation = new LarsonAnimation(0, kStripCount)
            .withSlot(kAnimSlot)
            .withColor(new RGBWColor(255, 0, 0, 0))
            .withSize(20)
            .withBounceMode(LarsonBounceValue.Front)
            .withFrameRate(Hertz.of(250));

    // Shooting: ColorFlow animation (overrides both fire slots)
    private final ColorFlowAnimation m_shootingAnimation = new ColorFlowAnimation(0, kStripCount)
            .withSlot(kAnimSlot)
            .withColor(new RGBWColor(255, 0, 0, 0))
            .withDirection(AnimationDirectionValue.Forward)
            .withFrameRate(Hertz.of(650));

    // ============================================
    // State machine
    // ============================================
    public enum LedState { IDLE, INTAKING, SHOOTING }

    private LedState m_state = LedState.IDLE;
    private LedState m_lastState = null;

    // ============================================
    // Constructor
    // ============================================
    public LedSubsystem(ShooterSubsystem shooter) {
        m_shooter = shooter;
        m_animTimer.start();
    }

    // ============================================
    // Periodic: handle state switching
    // ============================================
    @Override
    public void periodic() {
        SmartDashboard.putString("LED State", m_state.name());

        if (m_state != m_lastState) {
            m_animTimer.reset();

            switch (m_state) {
                case IDLE:
                    // Play both left and right fire regions
                    m_candle.setControl(m_leftFire);
                    m_candle.setControl(m_rightFire);
                    break;

                case INTAKING:
                    // Stop both fire slots before playing Larson
                    m_candle.setControl(new FireAnimation(0, 0).withSlot(kLeftFireSlot));  // clears left
                    m_candle.setControl(new FireAnimation(0, 0).withSlot(kRightFireSlot)); // clears right
                    m_candle.setControl(m_intakeAnimation);
                    break;

                case SHOOTING:
                    // Stop both fire slots before playing ColorFlow
                    m_candle.setControl(new FireAnimation(0, 0).withSlot(kLeftFireSlot));
                    m_candle.setControl(new FireAnimation(0, 0).withSlot(kRightFireSlot));
                    m_candle.setControl(m_shootingAnimation);
                    break;
            }

            m_lastState = m_state;
        }
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

    // ================================
    // Manual cycle for testing
    // =================================
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