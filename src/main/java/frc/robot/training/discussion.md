# Discussion Questions

After studying these examples, think about:

- HyperDrive only has one boolean (isEngaged). What would make you add a state enum to it?
- LightsaberHilt has 5 states. Could you reduce it to 3? What would you lose?
- The motivator sensor in HyperDrive prevents charging if broken. Where is this checked - in the command or subsystem? Why?
- BLOCKED state in LightsaberHilt is triggered by a sensor, not a request. Why is this handled differently than EXTENDING?
- Commands in LightsaberHiltCommands use sequences and waits. Why can't HyperDriveCommands use waitUntil(() -> hyperDrive.isReady())? (Hint: what happens when charging stops?)