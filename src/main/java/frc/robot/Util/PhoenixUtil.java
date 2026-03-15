package frc.robot.util;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj.DriverStation;

public final class PhoenixUtil {
    private PhoenixUtil() {}

    /**
     * Applies a Phoenix 6 configuration with retry logic.
     *
     * A single apply() on RIO CAN bus can return OK prematurely if the device
     * is still booting. Five retries catches transient startup failures.
     * Prints a DriverStation warning if all retries fail.
     *
     * @param deviceName Human-readable name shown in DriverStation warning
     * @param applyCall  Lambda that performs the apply() and returns StatusCode
     */
    public static void applyConfig(String deviceName,
            java.util.function.Supplier<StatusCode> applyCall) {
        StatusCode result = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; i++) {
            result = applyCall.get();
            if (result.isOK()) return;
        }
        DriverStation.reportWarning(
            "[PhoenixUtil] " + deviceName +
            " config failed after 5 retries: " + result.getName(), false);
    }
}