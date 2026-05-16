Flywheel Ramp Rate Test Protocol (CTRE Phoenix 6)

Purpose:
Determine the minimum closed-loop ramp period that reduces mechanical shock and current spikes without hurting spin-up time or shot recovery.

Setup Checklist (before testing)

Confirm:

☐ Robot securely restrained
☐ Fresh battery installed
☐ Same battery used across comparison tests
☐ Flywheel wheels installed
☐ Normal game-piece compression present
☐ Logging enabled (AdvantageKit / AdvantageScope recommended)
☐ Shooter target RPM chosen (typical scoring shot)

Recommended test RPM:

Example: 3200–3600 RPM typical scoring shot
Ramp Values to Test

Test in this order:

0.00
0.05
0.10
0.15

Optional refinement:

0.03
0.08
Test 1 — Spin-Up Behavior

Start from 0 RPM

Command shooter to target RPM

Record:

Metric	Value
Time to 90% RPM	
Time to ready tolerance	
Peak supply current	
Minimum battery voltage	
Mechanical harshness (subjective)	

Repeat 3 times per ramp value

Test 2 — Shot Recovery Performance (Most Important)

Spin to target RPM

Feed 1 game piece

Measure:

Metric	Value
RPM dip magnitude	
Recovery time to ready	
Second-shot readiness acceptable?	Yes / No

Repeat:

☐ single-shot test
☐ two-shot burst test
☐ three-shot burst test (if relevant)
Test 3 — Driver Responsiveness

Driver performs normal shot sequence

Evaluate:

Metric	Result
Trigger-to-ready delay noticeable?	
Auton timing affected?	
Shooter feels “soft”?	
Driver preference	
Signals to Log (AdvantageScope Recommended)

Log at ≥100 Hz if possible:

flywheel velocity
flywheel setpoint
supply current
stator current
battery voltage
applied output (voltage or duty cycle)

Optional:

follower velocity (temporary tuning aid)
Mechanical Stress Indicators (Watch Closely)

Signs ramp is helping:

startup jerk reduced
belts quieter at spin-up
current spike reduced
battery sag reduced

Signs ramp is too large:

shooter takes longer to become ready
recovery after shot slower
second shot inconsistent
auton timing slips
Selection Rule (Important)

Choose the smallest ramp value that:

✔ reduces startup shock
✔ reduces current spike or battery sag
✔ does NOT slow shot recovery
✔ does NOT delay readiness timing

Example outcome:

0.00 → violent startup
0.05 → smooth startup, fast recovery
0.10 → smooth but slower recovery

Choose: 0.05
Typical Final Values (Reference Only)

Most dual-motor FRC flywheels land near:

0.03 – 0.08 seconds

Values above:

0.12 – 0.15 seconds

usually indicate mechanical protection is compensating for something else.

If Ramp > 0.10 Is Required

Check:

belt tension too high
wheel compression excessive
hub slip
loose shaft interface
current limits mismatched
incorrect feedforward tuning
excessive inertia

Ramp should refine behavior, not fix root problems.

Final Ramp Selection

Chosen ramp value:

________ seconds

Tested on date:

________

Battery used:

________

Notes:

________________________________________
________________________________________
________________________________________