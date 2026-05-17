# Cyber Coyotes 2026 — Programming Retrospective Binder

## What this is

This binder is a code-level retrospective of the 2025-2026 FRC season. It contains twelve retrospective documents — one per major file or coherent grouping of files — each examining what worked, what didn't, and what the team should do differently for 2027.

Unlike a typical season recap, these retrospectives are written *to be useful*, not to celebrate. Every finding should be concrete, every recommendation is actionable, and every "open question" is something a we can answer with the codebase and data. The goal is that next year's programming team can read these documents and start build season already knowing what to do differently. It provides an opportunity to make changes before the offseason event and learn going forward.

## How this binder is structured

Each retrospective follows roughly the same template:

1. **What this file/subsystem does** — plain English, drive-coach-readable.
2. **Architecture at a glance** — file structure and what each part owns.
3. **Decisions we made and why** — design choices with rationale, not just code description.
4. **What worked well** — patterns and behaviors to lift and reuse.
5. **What we'd reconsider** — concrete findings, including code-level debt and architectural questions.
6. **Open questions for the student owner** — drives the actual review conversation; answer these *in writing* before the binder is final.
7. **Lessons for next season** — what the program should carry forward.

The template flexed where the file demanded it. The vision retrospective is explicitly Part 1 of a vision *system* retrospective that continues into Robot.java. The commands retro is built around the annotations on each file. The PhoenixUtil retro opens with a direct answer to "good call or embed?" because that was the question that motivated it. The RobotContainer retro uses a Good / Bad / Ugly framing because that's what was asked for.

Read them as a set, not in isolation.

## The retrospectives (suggested reading order for a new student)

This is the order that builds understanding from the ground up — sensor data to subsystems to coordination to top-level wiring.

1. **`RETROSPECTIVE_indexer.md`** — the simplest subsystem, plus the team's most important pedagogical narrative: the state machine that became a status indicator. The model for the offseason (FSM) full state machine rewrite plan.
2. **`RETROSPECTIVE_intake.md`** — the file with the most concrete code-debt findings (fuel-pump command explosion, identical duplicate methods), plus genuine strengths (software soft limits, derived state).
3. **`RETROSPECTIVE_shooter.md`** — the strongest code in the codebase. A real FSM. CAN bus optimization. Safety guards. This is the *answer key* for the patterns the team has been developing.
4. **`RETROSPECTIVE_vision.md`** — Part 1 of the vision system retrospective. The data provider layer, the single-point-of-fusion architecture, alliance-aware tag validation. Pairs with Robot.java.
5. **`RETROSPECTIVE_robot.md`** — Part 2 of the vision system retrospective. Where the pose-fusion math lives — 25 lines that are arguably the most important in the codebase.
6. **`RETROSPECTIVE_drivetrain.md`** — extending the CTRE generator output. The `resetPoseFromVisionCommand` is the strongest single command in the codebase.
7. **`RETROSPECTIVE_commands.md`** — coordination code. The late-season `AlignAndShoot` refactor and what was left behind. The "abstraction added but old code not deleted" pattern at its most visible.
8. **`RETROSPECTIVE_auton.md`** — 1143 lines of routine duplication. The headline finding (parameterization opportunity) is the largest offseason refactor in the codebase.
9. **`RETROSPECTIVE_robotcontainer.md`** — where the wiring lives. The auto chooser tells you which routines actually shipped (about 7 of the 20+).
10. **`RETROSPECTIVE_constants.md`** — file architecture: nested namespacing done right, with notes on tuning history clutter and the Vision section as the documentation standard.
11. **`RETROSPECTIVE_utilities.md`** — PhoenixUtil. A small file that does one thing well, and a teaching example of when utility extraction is the right call.
12. **`RETROSPECTIVE_crosscutting.md`** — patterns that appeared across multiple retros. Read after the others; this is the team-level retrospective that lives above the file-level ones.

Plus the prioritized backlog:

- **`OFFSEASON_BACKLOG.md`** — every "Lessons for next season" item compiled, ordered, and scoped into a workable backlog of issues for offseason and pre-2027 work.

## Reading paths for different audiences

### For a new student programmer joining the team

Read in the order above, in roughly that priority. The indexer, shooter, and vision retros are the highest-density teaching material. Don't skip Robot.java — it's short and contains the most important math in the codebase.

The cross-cutting findings document is best read *after* you've absorbed several individual retros, so you can see the patterns connecting them.

### For judges (impact, design, innovation awards)

1. **`RETROSPECTIVE_README.md`** (this file) — establishes the rigor and scope of the team's self-review.
2. **`RETROSPECTIVE_shooter.md`** — the technical sophistication is real and visible. The state machine, the CAN bus optimization, the interpolating distance lookup, the safety guards.
3. **`RETROSPECTIVE_drivetrain.md`** — the `resetPoseFromVisionCommand` is the kind of engineering maturity that demonstrates the team is thinking about real failure modes, not just happy paths.
4. **`RETROSPECTIVE_robot.md`** — the vision pose-fusion math (omega gate, distance² std-dev, theta-std-dev infinity) is sophisticated for a high-school team.
5. **`RETROSPECTIVE_crosscutting.md`** — pulls the themes together; demonstrates the team's ability to do *system-level* self-review.

For judges' interviews, the talking points the team should be ready with: the indexer FSM origin story (intended FSM, became status indicator, planned rewrite as deliberate practice), the late-season `AlignAndShoot` refactor, the build-team feedback about "too much on software" and how the code shows it, and the offseason backlog as evidence of continuous-improvement discipline.

### For next year's programming mentor

Start with `RETROSPECTIVE_crosscutting.md` and `OFFSEASON_BACKLOG.md`. The crosscutting document is the synthesized lessons; the backlog is the work plan. Then read the individual retros only for the files you're about to touch.

The recurring patterns to internalize: (1) IO/Hardware abstraction earned its keep, sim didn't, (2) "abstraction added but old code not deleted" is the most common code-debt pattern, (3) missing logging in fusion/decision code is the biggest debugging gap, (4) test commands shipping in production is universal and pruneable.

### For next year's programming lead (student)

Read the indexer retro twice. The FSM rewrite is your deliberate-practice offseason project, and the shooter is your model. Then read the auton retro, the commands retro, and `OFFSEASON_BACKLOG.md` for what to work on between now and kickoff.

The most important lesson in the entire binder: **complexity must earn its place.** That principle shows up in every retro, positive and negative. The patterns that worked (the shooter's FSM, the slide's soft limits, the vision's single-point-of-fusion) earned their complexity through real engineering problems. The patterns that didn't (the indexer's status enum, the fuel-pump command explosion, the duplicate alignment logic) accumulated complexity without earning it.

### For drive coach and operator

The retros aren't primarily for you, but two are worth reading:

1. **`RETROSPECTIVE_robotcontainer.md`** — every button binding is documented. This is the controller layout reference.
2. **`RETROSPECTIVE_drivetrain.md`** — the `back` button on either controller does the pose-reset-from-vision recovery. Know what it does and when to use it.

## Cross-references between retros

Some retros explicitly pair:

- **Vision + Robot.java.** The vision subsystem is the data provider; Robot.java owns the actual fusion math. Read them together.
- **Drivetrain + Robot.java.** Robot.java's vision fusion writes to the drivetrain's pose estimator. The drivetrain's `resetPoseFromVisionCommand` reads from the same Limelight that Robot.java reads from. They share the omega-filter principle.
- **Commands + RobotContainer.** RobotContainer binds the commands. The commands retro and the RobotContainer retro both note the same patterns from different angles.
- **AutoRoutines + RobotContainer.** RobotContainer's auto chooser is the source of truth for which routines from AutoRoutines.java actually shipped. About a third did.
- **Indexer + Shooter.** The shooter's real FSM is the model the indexer's status-indicator-enum should grow into. Read the indexer retro's offseason rewrite plan alongside the shooter's state machine implementation.

## How to use this binder

For students:
- Read your subsystem's retro carefully.
- Answer the "Open questions for the student owner" in writing, in the same file, before the binder is final.
- Pick one or two items from the "Lessons for next season" that resonate with your experience and write a paragraph about *why* it matters to you.

For the team as a whole:
- Use `OFFSEASON_BACKLOG.md` as the prioritized work list for May–December.
- Re-read `RETROSPECTIVE_crosscutting.md` at the start of build season — the recurring patterns identified there are the ones most likely to recur in 2027 if not deliberately interrupted.
- Use the individual retros as code-review reference material when 2027 PRs touch the same subsystems.

## What this binder is not

- It is not a celebration of the 2026 season. It's a clear-eyed review of what worked and what didn't.
- It is not a critique of any individual programmer. The findings are about code and patterns, not people. People who own subsystems should read the retros as professional review of their work, not as personal feedback.
- It is not exhaustive. LED was descoped during the season and is not retro'd. SysId routines are mentioned but not deeply analyzed. The `Telemetry.java` and `GameDataTelemetry.java` files are referenced but not retro'd.
- It is not the last word. The "Open questions" sections are honest gaps — answered by students with the codebase and match logs in hand, not assumed away.

## Version history

This binder was written in the post-season window of May 2026 by Scoy (programming mentor) in collaboration with Claude (AI assistant), reviewing the `2026-Rebuilt` codebase at the state it was in at the end of the FIRST AGE: Rebuilt season.

The team had used AI more this year than any prior year, and these retrospectives reflect AI-assisted code review at the file level — Claude reading the actual source, identifying patterns, surfacing findings — with mentor-led framing of which findings matter and why. The collaboration is itself part of the team's 2026 story.
