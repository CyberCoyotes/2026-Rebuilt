# Recording Motor Positions with Phoenix Tuner X

**For:** Cyber Coyotes
**Applies to:** Any TalonFX motor (Kraken X44, Kraken X60, Falcon 500)
**Tool:** CTRE Phoenix Tuner X (installed on the laptop)

Use this procedure any time you need to find what encoder value (in rotations) a mechanism is at for a specific position — extended, retracted, a hood angle, etc. — and save that number into `Constants.java`.

> **Safety rule:** Always use the lowest voltage that actually moves the mechanism. Start at 1 V. You should never have to exceed 3 V during this procedure.

---

## Before You Start

- [ ] Robot wheels are "clear" on a cart or safely supported. Have someone double check
- [ ] **Bumpers on/off** depends on the mechanism and if bumpers are part of the poosition you are trying to find
- [ ] A second person is watching the mechanism while you operate Tuner
- [ ] You know which motor ID you are tuning (check `Constants.java` for the ID)
- [ ] Phoenix Tuner X is installed on the drive laptop (`Phoenix Tuner X` in the Start menu)

---

## Step 1 — Connect to the Robot

1. Connect the laptop to the robot's radio **or** USB-B cable directly to the roboRIO.
2. Open **Phoenix Tuner X**.
3. In the top-left dropdown, select the connection type:
   - **WiFi** → select `10.48.29.2` (your team number) **OR**
   - **roboRIO USB** → select `172.22.11.2`
4. Wait for the device list to populate. You should see your motors listed by name and CAN ID.

> If no devices appear, check that the robot is powered on (main breaker is on) and the connection type matches how you are physically connected.

---

## Step 2 — Open the Motor

1. In the device list on the left, find the motor you want to tune by its **name** and **ID**.
   - Names come from Phoenix Tuner — check `Constants.java` for the ID if you are unsure.
2. Click the motor to open its detail page.
3. Click the **Self-Test Snapshot** button (top right) once to confirm the device is responsive.
   - You should see a green status indicator and firmware version.

---

## Step 3 — Set Up the Signals View

You need to watch the **Position** signal in real time as you move the mechanism.

1. Click the **Signals** tab (along the top of the motor detail page).
2. Find **Position** in the signal list — it is in rotations.
3. Pin it to the top by clicking the star/pin icon next to it.
4. Watch the value update live as you move the mechanism by hand (it may read 0.0 until the motor is powered).

> **Tip:** You can also watch `Velocity` to confirm the motor is actually moving when you apply voltage. 

---

## Step 4 — Move the Mechanism to the Target Position

This step uses Tuner's built-in **Control** tab to send a low-voltage command directly to the motor, bypassing the robot's normal enable state. You may need to **Enable** the **Driver Station**

1. Click the **Control** tab.
2. In the **Control Mode** dropdown, select **Voltage**.
3. Set the voltage to **`1.0`** (one volt). Do not go higher than 3.0 V during positioning.
4. Click **Enable Control** (orange button). The motor will now respond to the voltage you set.
5. Use the **+** and **−** buttons (or type a value) to inch the mechanism toward the position you want:
   - Positive voltage → forward direction
   - Negative voltage → reverse direction
   - If the mechanism does not move at 1 V, try 1.5 V or 2 V
6. Switch to the **Signals** tab to read the current position, then switch back to Control to nudge further.
   - Or open two Tuner windows side by side if your laptop supports it.
7. When the mechanism is at the exact position you want, **click Disable Control immediately**.

> **Do not leave Control enabled unattended.** Click Disable as soon as you have the reading you need.

---

## Step 5 — Read and Record the Position

1. With Control disabled, click the **Signals** tab.
2. Read the **Position** value. It will be in **rotations** (e.g., `8.42`, `44.40`, `0.00`).
3. Write it down or screenshot it.
4. Repeat Steps 4–5 for any other target positions you need (e.g., extended, retracted, angles).

---

## Step 6 — Enter the Value in Constants.java

1. Open `src/main/java/frc/robot/Constants.java` in VS Code.
2. Find the subsystem block for the motor you just tuned (e.g., `Intake`, `Shooter`).
3. Update the constant with the value you recorded:

```java
// Example — intake slide positions
public static final double SLIDE_RETRACTED_POS = 0.0;     // confirmed at hard stop
public static final double SLIDE_EXTENDED_POS  = 44.40;   // recorded from Tuner
public static final double SLIDE_MAX_POS       = 44.454;  // soft limit — slightly past extended
```

4. Add a short comment on the same line with what the position represents and the date if it was recently tuned.
5. If a constant previously had a `// TODO` comment, remove it or replace it with `// Tuned MM/DD`.

---

## Step 7 — Verify in Code

After updating `Constants.java`, do a quick sanity check before deploying:

| Check | What to look for |
|---|---|
| `SLIDE_MAX_POS` ≥ `SLIDE_EXTENDED_POS` | Extended position should never exceed the soft limit |
| Retracted position is `0.0` | We zero the encoder at startup — retracted should always be `0.0` |
| No two positions are the same value | If two constants are identical, one is probably wrong |
| Tolerance constant is realistic | `±0.05` rotations is tight; `±0.25` is loose — pick based on mechanism slop |

---

## Troubleshooting

| Problem | What to check |
|---|---|
| No devices appear in Tuner | Robot power on? Correct IP/connection type selected? |
| Motor beeps but doesn't move | Check for a soft limit enabled in the config — may need to zero encoder first in Tuner |
| Position reads `0.0` and never changes | Encoder may not be initialized; deploy code once to run `setPosition(0.0)` at startup |
| Motor lurches instead of moving smoothly | Voltage is too high — drop to 0.5 V and approach slowly |
| Position jumps around at rest | Normal CAN noise at the last decimal — round to 2 decimal places in Constants |
| "Device not responding" in Tuner | CAN ID may be wrong; check Constants.java and Tuner device list match |

---

## Quick Reference: What the Signals Mean

| Signal Name | Unit | What it means |
|---|---|---|
| Position | rotations | How far the motor shaft has turned since last zero |
| Velocity | rotations/sec | How fast the shaft is spinning right now |
| SupplyCurrent | amps | Current drawn from the battery |
| StatorCurrent | amps | Current through the motor windings (torque proxy) |
| DeviceTemp | °C | Motor temperature — above 70°C is getting warm |

---

*See also: `flywheel-tuning-guide.md` for velocity PID tuning, `TUNING.md` for full system tuning protocol.*
