# Architecture and code review — October 28, 2026

## Assessment and scope

The mechanism-command/request/vendor-wrapper separation is worth keeping. The framework is not yet a dependable foundation for future seasons because its lifecycle, units, ownership, and failure contracts are inconsistent. Adding more base classes before addressing those contracts will spread the inconsistencies.

This is a source review of the current working tree, focused on startup, composition, commands, motor IO/configuration, shooting, swerve, vision, logging, simulation, and representative math utilities. It is not a hardware acceptance test or an exhaustive audit of every borrowed visualization/camera helper. Hardware-dependent conclusions are identified below. Robot source was not changed.

Validation: `gradlew.bat test --offline -x createVersionFile` succeeded using `C:/Users/Public/wpilib/2026/jdk`. The default shell initially used Java 8 and failed. Importantly, `src/test` contains directories but no test source files; the successful test task does not establish behavioral coverage. `compileJava` was up to date, so this was not a clean compilation. Version generation was excluded to avoid changing generated source during review. No checked-in CI workflow was found.

## Highest-priority findings

### 1. TalonFX profile constraints are updated on an unused request — P1, confirmed

`MotorIOTalonFX.java:239–243` writes velocity/acceleration into `magicOutput`, a DynamicMotionMagicVoltage. `setPositionTrapezoidal()` at line 313 instead sends `magicVoltage`, a MotionMagicVoltage. Consequently per-command speed overrides do not reach the submitted request. Intake deploy and jiggle request different speeds but this wrapper uses the configured static profile instead. TalonFXS submits its dynamic request correctly, so identical framework calls have different semantics by vendor implementation.

Use one intentional profile path and test the submitted position, slot, velocity, and acceleration. The prior repeated-full-configuration issue has been removed from this TalonFX path, but the replacement is incomplete.

### 2. Vision source selection can reject both available sources — P1, confirmed

`VisionHandler.java:168` invalidates Quest seeding when tracking is lost. Quest measurements require `isQuestSeeded` at line 179. Once Quest reconnects and tracks, AprilTag processing returns immediately at lines 92–95, regardless of seeding. Automatic tag seeding is commented out. Thus after a tracking interruption, tracking can be green while no source contributes corrections until an explicit known-pose reset.

Select sources by usable localization state, not connection/tracking alone. Represent disconnected, tracking-unseeded, usable, and stale states explicitly. Test disconnect → reconnect → tracking → reseed. If exclusive Quest priority is intentional, surface the resulting no-correction state clearly.

### 3. Shooting readiness is a one-time latch and excludes aiming — P1, confirmed

`Superstructure.java:26` checks only hood/flywheel. The `waitUntil(...).andThen(feed())` sequences at lines 86, 106, and 126 never recheck readiness after feeding starts. Wheel-speed droop or a changed hood target does not pause feeding. Autonomous shooting starts aiming and readiness waiting in parallel, so feeding can start before heading alignment.

Define explicit readiness reasons: valid shot, referenced mechanisms, speed recovered, hood settled, heading valid, and usable pose when required. Gate feed continuously, with stability time and separate enter/exit thresholds to avoid chatter. Provide an explicit operator override instead of weakening the normal predicate.

### 4. Mechanism simulation is bypassed by telemetry overrides — P1 for simulation, confirmed

`BaseMotorSubsystem.java:85–86` advances motor simulation inside `periodicTelemetry()`. Concrete mechanism subsystems override that method without calling the parent. Attached flywheel, feeder, roller, and hopper models therefore do not advance through that lifecycle. Hood and intake deploy also do not attach their defined models in their current factories. Power construction and updates are commented out in Robot.

Move physics advancement into a dedicated simulation lifecycle that cannot be replaced by a telemetry override. Keep input sampling, physics, output calculation, and logging separate. Verify a voltage command changes position/velocity and current before trusting simulated autonomous or power behavior.

### 5. Deactivation does not establish a safe output — P1 if used operationally, confirmed

`BaseSubsystem.deactivate()` only changes a flag. Continuous `run()` actions stop executing and end when inactive, but do not neutralize an already-latched controller output. One-shot requests have already finished. SysId's active selection is only evaluated at initialization. Public motor access and direct output methods bypass the gate. Telemetry also stops, hiding feedback during the fault.

Distinguish construction-time hardware omission from runtime inhibition. Give runtime inhibition an explicit per-mechanism safe action, cancel ownership appropriately, and continue health telemetry. A gravity-loaded mechanism may need a different response from a roller. Never describe a skipped write as a guaranteed motor stop.

### 6. Configuration failures are silently discarded — P1, confirmed

TalonFX leader setup ignores the apply status. Its follower status checks at lines 346 and 350 are commented out. SPARK leader construction ignores configure status. `SwerveSubsystem` catches AutoBuilder configuration exceptions and prints only a generic message, then startup continues into autonomous loading.

Make setup return an inspectable success/failure result. Use bounded retries, report device ID/bus/operation/status, latch a driver-visible fault, and prevent affected actions when required configuration failed. Preserve exception details. Do not add unbounded retries or periodic configuration loops.

### 7. Hood encoder zero is assumed at boot — P1 pending hardware confirmation

`HoodSubsystem` resets a relative position to `kHomeAngle` unconditionally in its constructor. `home()` commands that numerical zero; it is not a sensor-based homing procedure. Soft limits then trust that assumed reference. If the robot boots with the hood away from home, both the commanded position and limit reference can be wrong.

Document and enforce the startup mechanical condition or add a real reference procedure/absolute sensor. Track whether the mechanism is referenced. Do not enable position-dependent shooting before that state is valid.

## Units and physical configuration

### 8. Typed angles still mix rotor and mechanism coordinates

Hood gearing is about 162:1, but its feedback ratio is commented out, shot targets are 0–20 rotations, and its limit is 22 rotations. Those appear to be motor coordinates. The simulation uses gearing but receives the same values as mechanism angles. `kEpsilon = Rotations.one()` is plausibly a small hood tolerance in rotor units, but a full turn in mechanism units. This is an unresolved contract, not proof that the physical hood turns 22 revolutions.

Choose mechanism angle as the subsystem public convention. Convert the shot table, gains, constraints, tolerances, and limits together when changing feedback scaling. If calibration must remain in rotor units temporarily, label it explicitly rather than calling it a hood angle.

### 9. Hopper geometry and commanded travel disagree

`HopperConstants` declares 7 inches maximum travel, a 0.5-inch radius, and a converter based on that radius. The IO uses a 1:1 encoder conversion, while `extend()` commands 45 rotations. The same converter maps that to approximately 141.4 inches, so `hopperExtended` cannot describe the declared 7-inch position consistently. Gearing is still a TODO. The real linkage/transmission must determine the correction.

Command typed linear travel through one verified conversion. Derive controller limits and simulation bounds from that same travel. Telemetry should distinguish requested extension from measured extension and use a tolerance rather than exact equality.

### 10. Minion factories publish Kraken identity

`MotorIOTalonFXS.generateMinion()` passes `MotorConstants.kKrakenX60` to the wrapper. This makes `getConstants()`, `getDCMotor()`, and free-speed metadata wrong even though commutation selects a Minion. Correct every Minion overload and test factory metadata. A factory name should be a reliable physical identity.

### 11. CAN bus support is overstated

Hopper's PortConfig names the second CANivore, but SPARK construction uses only `port.kPort`. The common configuration implies bus selection that this implementation does not honor. Validate unsupported combinations or use vendor-appropriate address types. Confirm actual wiring separately.

### 12. Current-limit terminology should remain vendor-specific

The SPARK builder correctly documents smart/secondary limits, but Hopper passes constants named supply/stator. Those names imply CTRE semantics. Rename the mechanism settings to what REV actually accepts. A cross-vendor abstraction should not suggest equivalence for physically different controls.

## Framework architecture for future seasons

### 13. Robot is a global service locator

Public static subsystem fields require Robot construction in a specific order. Superstructure, ShotGenerator, Controls, and library code reach through Robot. The comments in RobotContainer say it builds the subsystem graph, but Robot actually does. This makes isolated tests and alternate robot configurations unnecessarily difficult.

Make RobotContainer the composition root with private final subsystem references. Inject only required dependencies into Superstructure. Inject pose/speed/alliance suppliers into shooting and vision. Constructor injection here needs no dependency-injection framework.

### 14. Reusable library packages depend on seasonal robot code

BaseSubsystem imports RobotConstants, PortConfig imports RobotConstants, camera IO imports Robot/VisionConstants, and vendor wrappers import Robot. Copying `frc.lib` into next season therefore brings hidden robot-specific requirements.

Supply log policy, transforms, layouts, and runtime mode through constructors/configuration. Keep game targets, field layout, shot tables, and control policy in seasonal code. Treat independence from `frc.robot` as an enforceable library boundary.

### 15. IO is a vendor facade, not yet a replay boundary

Subsystem getters read live wrappers directly and populate logging records afterward. Replaying an input record would not make those getters consume replayed feedback. Robot extends TimedRobot, logging start is commented out, and no replay source is installed. REPLAY exists as an enum but is not an implemented operating mode; current getRobot() also forces SIMBOT on desktop.

Choose the intended scope. A lightweight facade is useful if replay is not needed. If replay is a requirement, make IO populate an input snapshot and have control logic read that snapshot consistently. Install the appropriate logging lifecycle and replay source. Avoid maintaining an advertised mode that cannot run.

### 16. Request history can disagree with hardware

MotorIO stores the mutable request object by reference after applying it. External mutation can change the reported target without another hardware write. Direct `setPosition()` calls bypass request history entirely; intake jiggle cleanup and `collapseHopper()` use that route. atSetpoint and getSetpoint can then refer to a different request from the active hardware control.

Centralize output submission and store a stable description of the last submitted request. Restrict vendor escape hatches. Separate desired, submitted, and measured state when reporting failures.

### 17. A double cannot encode both linear and angular request targets safely

LinearPositionRequest reports meters from getBaseValue(), but uses the same position RequestType as angular requests. BaseServoSubsystem.getSetpoint() interprets every position request's base value as radians. The linear request API is documented as usable from this base class, so this is a latent public-API bug.

Keep target quantities typed end to end, or separate linear and angular subsystem interfaces. Do not infer the physical dimension from a broad position-control enum.

### 18. Overloads hide different command lifetimes

`applyRequest(MotorRequest)` returns runOnce, while `applyRequest(Supplier<MotorRequest>)` runs continuously. The spelling difference is small; the ownership/lifetime difference is substantial. Hopper extend/collapse finish immediately while the controller continues holding, whereas hood commands remain scheduled.

Use explicit names such as submitOnce, runRequest, and moveToPosition. Document completion and interruption behavior. One-shot holding can be intentional, but code review should not require tracing overload resolution to discover it.

### 19. Lifecycle inheritance is too easy to bypass

`periodicTelemetry()` performs functional work, and `periodic()` is not final despite documentation asking subclasses not to override it. This already caused the simulation bug. Prefer final orchestration methods with narrow hooks for input refresh, control, telemetry, and simulation. Keep base-constructor calls to overridable methods minimal; BaseVelocitySubsystem currently invokes stop() during construction.

### 20. Generic fluent APIs promise types they cannot guarantee

`<T extends BaseSubsystem> T deactivate()` allows callers to infer an unrelated subclass and fail at runtime. Generic getMotor()/withSimulation() declarations have a similar caller-chosen-type smell. Return a truthful base type, use covariant concrete returns, or use a justified self-type pattern. Avoid unchecked casts merely to support chaining.

### 21. Missing commands should remain absent

BaseSubsystem.getCurrentCommand()/getDefaultCommand() return newly created Commands.none() instead of null when absent. This changes the inherited contract, makes null checks misleading, creates allocations, and gives callers a command that was never scheduled. Preserve the framework API; add an Optional accessor or a separate display-name helper if useful.

## Shooting and swerve details

### 22. Shot tables ignore their independent variable

ShotTable collects entry distances and discards them, interpolating by array index at one-meter spacing. Current samples happen to fit that assumption. Adding a measured 3.5-meter sample would silently move subsequent calibration points. Interpolate over explicit sorted distances; reject duplicates/nonfinite values and define behavior outside calibrated range.

Return a typed ShotParameters/ShotSolution record instead of double[3]. Include angle, angular velocity, flight time, validity, and calibration-range status. Avoid silently treating an endpoint-clamped long shot as calibrated.

### 23. Shot calculation is coupled to mutable global state

ShotGenerator exposes public mutable target/result fields and reads Robot.swerve directly. Superstructure changes mode while periodic updates continue. Its normal update runs after CommandScheduler, so continuous commands generally consume the preceding loop's solution; setTarget explicitly updates immediately, creating two timing paths.

Compute one immutable solution from a coherent pose/speed snapshot before command execution. Inject calibration and target policy. Keep the pure solver independently testable. Offset shooter velocity and release delay are later modeling improvements, after the basic contracts are correct.

### 24. Autonomous heading lock stops correcting after first arrival

lockHeadingTarget() ends when within one degree. In autoShoot's parallel group, other commands continue and retain the group's drivetrain requirement, but heading corrections do not restart after drift. Provide separate aimContinuously and aimUntilAligned operations. Include heading readiness in feed policy. Reset heading PID state on entry to avoid derivative history from a previous target.

### 25. Auto-align helpers contain direction/state hazards

autoAlignXY and autoAlignYThenX negate positive PID corrections even though their field-centric request expects velocity toward the target. Verify both alliances and perspective conventions before use. Additionally the Y-first stage updates Y but never clears X on its shared mutable kAutoAlignRequest; a prior call can leave nonzero X in the request. Fully specify requests or make them command-local. Add explicit speed bounds and meaningful completion behavior.

### 26. Constants contain mutable controllers and requests

SwerveConstants holds stateful PIDController objects and SwerveRequests holds mutable request singletons. They are runtime state, not constants. Keep gain values/configuration in constants; own controllers and request instances in the subsystem or command that uses them. This also makes multiple instances and tests possible.

## Operational quality and smaller issues

- **Logging is currently disconnected:** Logger.start(), metadata, file receivers, and power wiring are commented out. Merely setting kLogToWPILog cannot enable the commented branch. README describes active power monitoring that the current startup does not construct. Make feature toggles control executable behavior and expose active configuration.
- **Control chooser is decorative:** Controls.build() publishes options but never reads getSelected() to update controlSet. With one option this is harmless, but it is not a working swappable-control design. Implement it when needed or remove the placeholder.
- **Conflicting held controls need policy:** spit or hopper collapse can interrupt an ongoing held shot/intake. A whileTrue command does not automatically restart merely because the button remains held after interruption. Decide priority and restart semantics, then test button transitions. The ordinary shoot/intake combinations are now mutually exclusive, which is an improvement.
- **Autonomous loading is fragile:** every Comp_ auto is constructed eagerly and the default is mandatory. A malformed optional file can affect startup. Validate deploy assets ahead of time, provide a safe DoNothing choice, report individual invalid routines, and guard normal operation from optional asset failures.
- **autoHasRan is permanent:** disabled preload stops changing the pose after the first autonomous. autonomousInit now resets the selected start pose, so this is not the former missing-start-reset bug; it is a disabled-preview/repeated-practice policy that needs a clearer name and explicit lifecycle.
- **Coordinate2d.getY() returns X:** this also corrupts vectorBetweenCoordinates and mirroring. Both Vector2d component constructors use atan(y/x), losing quadrant information. Replace with WPILib Translation2d where possible; otherwise use atan2 and add negative-axis tests.
- **PiecewiseRegression edge cases:** a negative input between -delta and zero extrapolates because integer conversion truncates toward zero. Nonpositive spacing, nonfinite input, null samples, and external mutation of the stored array are not handled. getResidual() returning 1.0 invents a statistic rather than declaring it unavailable.
- **SPARK idle requests reconfigure hardware:** coast()/brake() configure on every invocation. Repeated idle requests can become periodic configuration writes. Cache neutral mode or separate persistent idle-mode configuration from ordinary zero-output commands.
- **Health checks should cover freshness and validity:** Quest timestamps are checked for finiteness/reset ordering, but not maximum age/future timestamps; Quest pose values are not checked for finiteness before fusion. Validate these at the ingestion boundary. Reset seeding also marks success immediately after requesting a reset; that flag should distinguish requested from confirmed if confirmation is available.
- **Telemetry names overclaim measured state:** intake deployed is based on target, hopperExtended compares target to an exact maximum, and flywheel isWarmingUp means nonzero target, not actual acceleration. Name intent and measured readiness separately. Include units in numeric keys; avoid generic dashboard names such as Current Rotations.
- **RobotState is stale scaffolding:** update methods and call sites are commented out while getters still return initialized state. Remove it from active expectations or give it one authoritative producer.
- **Build dependency mismatch:** build.gradle declares AdvantageKit runtime and processor 4.0.0 while vendordep declares 26.0.2. Align them deliberately; a successful dependency resolution does not prove runtime/processor compatibility.
- **Development-only deployment settings:** unconditional unauthenticated remote JMX and a hardcoded robot address live in the default deploy configuration. Gate diagnostic services behind an explicit development profile. Derive addresses from configuration.
- **Stale deploy assets:** deleteOldFiles=false can retain removed autos/files on the robot. Define an intentional cleanup/manifest policy so last season's or renamed assets cannot remain selectable unnoticed.
- **Documentation accuracy:** BaseServo/BaseVelocity refer to nonexistent updateInputs(), Controls claims bindings live in RobotContainer in language that implies the mappings are elsewhere, Robot lifecycle comments claim current-limit changes absent from the bodies, and IntakeDeploy's comment says extension is negative while its target is positive. Remove obsolete comments and commented-out implementations; Git preserves history.
- **Small cleanup:** private dependencies should usually be final; remove unused imports and wildcard imports where they obscure ownership; prefer descriptive telemetry/command names over repeated SetPosition; name epsilon as positionTolerance/velocityTolerance; remove impossible primitive Objects.requireNonNull checks; correct inclusive-comparison Javadocs; use a formatter rather than hand-maintaining mixed indentation. These follow correctness work, not precede it.

## What is working well

1. Mechanism APIs expose intent: intake, shoot, home, feed. Keep vendor control objects out of Superstructure.
2. Command composition generally carries subsystem requirements, and roller defaults explicitly command zero output.
3. Typed request tolerances validate finite nonnegative values and request comparisons reject invalid feedback. This is a useful boundary to extend to setpoints and constraints.
4. Fixed hood requests no longer share the previous static mutable target. Servo hold now captures the position when scheduled.
5. Heading continuous input now uses radians consistently. Target-lock completion now compares desired heading to actual heading.
6. Passing selects a distinct calibration table, and moving-shot aiming uses the compensated target.
7. AprilTag timestamps are deduplicated and checked for age. Quest mounting transforms are explicit and reset-time ordering is tracked.
8. Motor configuration builders separate construction from use, retain followers, and snapshot lists/configurations in useful places. TalonFXS follower setup retains failure cleanup. Restore equivalent failure handling in TalonFX.
9. Controller mappings, device addresses, mechanism configuration, and calibration tables have recognizable homes. Refine those boundaries rather than replacing the whole project.

The October 5 report is historical: shared hood requests, zeroed optional profile constraints, several command-helper bugs, heading-unit mismatch, missing tag deduplication, and unused roller/kicker current-limit constants should not be repeated as current findings.

## Suggested migration order

1. Fix the submitted TalonFX profile request, vision recovery state, continuous feed readiness, and configuration failure reporting. Verify hood/hopper referencing and units on the actual mechanisms.
2. Make lifecycle orchestration reliable: safe inhibition, dedicated simulation stepping, consistent input sampling, explicit command lifetimes, and one output submission path.
3. Move subsystem construction into RobotContainer and inject dependencies. Remove seasonal imports from reusable libraries. Return typed immutable shot solutions and targets.
4. Add a small set of high-value tests: submitted IO profile constraints; reference/safe-disable behavior; readiness loss during feeding; vision loss/recovery; shot interpolation at irregular distances; command interruption and held-button transitions; simulation motion/current; all autonomous assets constructing with valid requirements.
5. Add CI using the intended JDK and build dependencies, plus formatting. Keep the required behavioral tests focused on library contracts and match-critical transitions.
6. Maintain a season migration checklist: hardware identity and bus map, units/gearing/reference procedure, field frame/layout, calibration ranges, vendor versions, auto assets, dashboard topics, enabled logging, and acceptance checks. Record old assumptions explicitly instead of carrying dormant features forward.

Promote helpers into the reusable library after they have a clear contract and a demonstrated second use. More abstraction is not automatically more adaptability; fewer hidden assumptions are.
