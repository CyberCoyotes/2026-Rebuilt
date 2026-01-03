# 🎓 Training Folder

**This folder is temporary.** It exists for pre-season Git and subsystem practice. Everything here will be deleted after kickoff when we build real subsystems.

## Purpose

This folder lets you practice:
1. **Git workflow** — branching, committing, pushing, pull requests, issues
2. **Subsystem structure** — how FRC subsystems are organized
3. **Command patterns** — how commands interact with subsystems
4. **Code review** — giving and receiving feedback

## Rules

### ✅ DO
- Create your own branch: `yourname-subsystem`
- Make your subsystem something fun (HyperDrive, CandyLauncher, PoolNoodleYeeter, etc.)
- Use fake CAN IDs (pick numbers 50-99 to avoid conflicts)
- Follow the same patterns as real subsystems
- Ask questions in your PR!

### ❌ DON'T
- Merge directly to `main` — always use a PR
- Use real CAN IDs (those are reserved for actual hardware)
- Worry about perfect code — this is for learning
- Skip the PR process "because it's just practice"

## Workflow

```
1. Create your branch
   git checkout -b yourname-candylauncher

2. Create your files in this folder
   training/
   ├── CandyLauncher.java        (your subsystem)
   └── CandyLauncherCommands.java (your commands)

3. Commit often with good messages
   git add .
   git commit -m "Add basic motor control to CandyLauncher"

4. Push your branch
   git push -u origin training/yourname-candylauncher

5. Open a Pull Request on GitHub
   - Base: training-main (NOT main!)
   - Compare: your branch
   - Fill out the PR template

6. Get code review, make changes, get approved

7. Merge your PR (squash and merge)
```

## File Naming

Put your name or initials in your filename to avoid conflicts:

```
training/
├── README.md                    (this file)
├── HyperDrive_Chewie.java      
├── HyperDriveCommands_Chewie.java
├── PoolNoodleYeeter_R2D2.java
└── PoolNoodleYeeterCommands_R2D2.java
```

## After Kickoff

This entire folder gets deleted (or left for funsies). Your real subsystems will live in:
```
src/main/java/frc/robot/subsystems/
src/main/java/frc/robot/commands/
```

The patterns you practice here transfer directly — just with real mechanisms and real CAN IDs.

---

**Questions?** Ask in person, email, or tag a mentor in your PR!