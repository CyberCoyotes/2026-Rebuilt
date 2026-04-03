
# START HERE FRIDAY Overview

## More Updates
More comment clean-ups last night and this morning. I've included some inline comments where appropriate and reference docs when more information is needed in the [Constants.java](https://github.com/CyberCoyotes/2026-Rebuilt/blob/500-adjust-the-shooter-subsystem-with-the-new-mechanicals/src/main/java/frc/robot/Constants.java)

In the [docs](https://github.com/CyberCoyotes/2026-Rebuilt/tree/500-adjust-the-shooter-subsystem-with-the-new-mechanicals/src/main/java/frc/robot/docs) folder, I updated and renamed the tuning guides for clarity. There is a general "tuner-x-guide_position-recording.md" there and some specific tuning guides for things like the flywheel.

## Basic Flow Today

1. Pull/fetch the latest changes from GitHub Desktop before doing anything.
2. Read the TODOs in Constants.java to know what needs tuning.
3. Use Tuner X to find the best values through live testing — this writes directly to the motor, not the code.
4. Hard-code the tuned values into Constants.java. Tuner X values disappear on the next deploy if you skip this.
5. Deploy the code and confirm the behavior matches what you tuned.
6. Add an inline // tuned comment and remove the TODO when a value is confirmed.
7. Only work on one subsystem at a time. Commit your changes before moving to the next one.

## Reminders

- Robot safety: robot on blocks or clear carpet space, someone on the disable button, never reach into the robot while enabled.
- Text or email if you need help. Don't let anyone rush you — follow the protocol.

## Subsystems

Mr. Smith can guide you as to priority (guessing Shooter first)