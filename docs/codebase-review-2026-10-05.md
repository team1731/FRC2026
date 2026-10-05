# FRC2026 codebase review — October 5, 2026

The code has a useful structure and builds successfully, but important behavior remains
unfinished or inconsistent. I would prioritize autonomous integration, motor request
semantics, heading control, and shooting recovery before further PID tuning.

This review covers the current working tree, including uncommitted changes, not the
specific binary deployed at a competition. It examines the active robot lifecycle,
controls, all mechanisms, shooting, vision, hardware wrappers/configuration, logging,
simulation, supporting math, and deployed autonomous assets. Supporting imported
libraries received targeted inspection; this is not an exhaustive proof of every
third-party utility. No robot deployment or hardware test was performed. Files were
being edited during the review; findings describe the versions inspected.

`gradlew test --offline` passed: eight tests, covering DriveScalar and DriveSpeedLimiter.
The six autonomous files' referenced path files exist. The autonomous command-name
audit found `Shoot` and `TargetLock`; only `TargetLock` is registered as a NamedCommand.

## Highest-priority findings

### 1. Autonomous named shooting is not registered — major, confirmed

RobotContainer.java:26 creates `EventTrigger("Shoot")`, whereas the autos contain
`type: named` commands for Shoot (for example Comp_RightOverBumpX2.auto:28).
RobotContainer.java:32 only registers TargetLock through NamedCommands. Event triggers
and named commands are separate mechanisms. The named shooting stages therefore do
not resolve to the intended shooting command.

Do not simply register the current full teleop shoot command alongside TargetLock:
shoot also owns swerve, while these autos place Shoot and TargetLock in parallel.
Separate mechanism-only autonomous shooting from aiming so requirements do not overlap.
Audit the marker `LowerSqueezer` in LeftX2NoBump_1.path:151 as well: the registered event
is `LowerSqueeze`, so this particular marker has no matching trigger.

### 2. Heading control mixes degrees and radians — major, confirmed

SwerveConstants.java:55 configures continuous input [-180, 180] and tolerance 1.0,
documented as degrees. SwerveSubsystem.java:244 and :325 provide radians.
The controller therefore wraps over 360 radians, not one revolution, and the
nominal one-degree tolerance actually represents roughly 57 degrees. Near +/-pi,
it can take the long route. P=0.5 also yields only about 0.785 rad/s for a 90-degree
error before derivative effects, explaining slow target-lock rotation independently
of the corrected manual drive curve. Use one unit system throughout and retune the
heading gains in that system. Reset controller state when entering aiming commands.

### 3. TargetLock's completion error is always 180 degrees — major, confirmed

SwerveSubsystem.java:327 subtracts desiredAngle from targetAngle. DesiredAngle is
defined as targetAngle plus 180 degrees, so this error never represents the robot's
actual heading error. The command cannot finish through its one-degree condition.
External deadlines can hide this bug by interrupting it. Compare desiredAngle to
current robot heading instead, and ensure the final motor request is appropriate
when the command ends.

### 4. Hood commands share a mutable target — major, confirmed

HoodSubsystem.java:14 uses one static mutable kShotRequest. home() and the fixed-angle
setAngle() mutate it when commands are constructed, then capture the same object.
Constructing another command or executing a supplier-based shot changes the request
already captured by home and other commands. A subsequent home can command the latest
shot angle rather than zero. Use separate requests or set the target at execution
time for each command. The fixed-angle hood commands are also one-shot commands,
whereas supplier-based hood commands run continuously; this lifetime difference
needs to be deliberate in parallel groups.

### 5. Profile requests overwrite constraints with zero — major, confirmed

TrapezoidalPositionRequest.java:36 initializes optional velocity and acceleration to
zero objects. apply() checks for non-null and always calls updateTrapezoidalSpeeds().
Thus a request with no explicit speeds does not retain controller constraints as its
documentation claims: it replaces them with zeros. BaseServoSubsystem.setPosition()
uses this path, as do hopper extend/collapse and turret tracking. Make constraints
truly optional, and supply valid constraints for mechanisms that require them.

### 6. Full motor configurations are applied during periodic control — major, confirmed

MotorIOTalonFX.java:232 updates DynamicMotionMagicVoltage's dynamic fields, then calls
configurator.apply(cfg) on the full configuration. Every profiled supplier command
calls this again during execution. Hood aiming and intake deploy can therefore perform
configuration writes each scheduler cycle. MotorIOTalonFXS has the same pattern, and
SPARK profile updates also reconfigure the controller repeatedly.

These are configuration transactions rather than ordinary setpoint updates. They can
consume CAN bandwidth and stall the main loop, especially during CAN faults. This is
a concrete software candidate for the previously discussed input lag, not proof of
the cause of a particular match incident. Configure once, cache changed settings,
and use the dynamic request fields for ongoing profiled control. Check returned status
codes; several leader configuration and request results are currently ignored.

### 7. Moving-shot compensation is disconnected from aiming — major, confirmed

ShotGenerator computes compensatedTarget and exposes appliedTargetSupplier, but
Superstructure.java:79 aims with the original target supplier. Hood/flywheel parameters
are selected using compensated distance while swerve points at the uncompensated
target. The two halves of the solution disagree during motion. The trackTarget argument
is also unused. Feed the compensated target into aiming and make tracking behavior
explicit. The current solver also ignores shooter release delay and the translational
velocity caused by rotation about an offset shooter; those are later modeling work,
not the first issue to fix.

### 8. Feeding is not continuously gated on recovery or aim — major, confirmed

Superstructure.java:25 checks hood and flywheel readiness, but not heading. At :81,
waitUntil gates feeder startup once. Indexer and kicker continue feeding if the shooter
subsequently falls below target. This matches the earlier concern about short shots
during sustained passing and voltage sag. Recheck readiness continuously, with separate
pause/resume thresholds and a short stability interval. Include heading and pose
validity where necessary. Log the reason feeding is inhibited.

### 9. Mechanism simulations do not advance — major for simulation, confirmed

BaseMotorSubsystem.java:83 is the only caller of motor.simPeriodic(). Every concrete
motor subsystem overrides periodicTelemetry() without calling the superclass. Thus
the attached hood, flywheel, hopper, indexer, kicker, and roller models never advance
through the active subsystem loop. IntakeDeploy has no attached physics model at all.
The power simulator is wired correctly at a high level, but its mechanism-current
inputs are consequently not reliable. Put simulation advancement in a lifecycle hook
that subclasses cannot accidentally bypass, then test movement and current together.

## Additional functional bugs and incomplete behavior

- **Passing uses the scoring table.** ShotGenerator has one final scoring table even
  though ShotTable supplies a separate passing table. pass() changes the target but
  never changes the table. Select the appropriate table with the shot mode.
- **Auto-align translation points away from the setpoint.** SwerveSubsystem.java:289
  negates the positive PID correction. On blue, a robot behind a +X target receives
  negative X velocity. Also review field coordinates versus operator perspective on red.
  These helpers are not currently bound in the active driver configuration.
- **Fixed-request UntilAtSetpoint helpers end immediately.** BaseMotorSubsystem.java:57
  decorates a runOnce command with until(); the underlying command still finishes
  after its one-shot action. It does not wait for the motor to arrive. Use an explicit
  wait after application or a continuous command with a completion condition.
- **Servo stop does not hold the captured position.** BaseServoSubsystem.java:85
  creates and discards a new request with the current position; the command then applies
  a different default-zero request. It can drive toward zero instead of holding.
- **Combined driver triggers have overlapping ownership.** shoot and intake remain
  bound individually while their combined feedthrough trigger schedules another command
  using the same subsystems. For example, releasing intake while still holding shoot
  cancels feedthrough, but the plain shoot trigger has no new rising edge to restart it.
  Use mutually exclusive modes or one drive/shoot state machine.
- **DriveCommand is a scaffold.** execute() is empty and SwerveSubsystem still owns the
  actual default drive command. Its lockToTarget() sets a flag in an InstantCommand and
  clears it in finallyDo when that instant command immediately ends. It does not yet
  implement persistent target lock.
- **ContinuousConditionalCommand is not continuous.** Its DeferredCommand chooses a
  branch only when initialized. It also declares only the true branch's requirements,
  which is wrong if the false branch owns additional subsystems. It currently has no
  active call sites.
- **SysId null checks occur too late.** BaseSubsystem's dynamic/quasistatic builders
  dereference sysIdRoutine before Commands.either can evaluate the null guard.
- **SPARK reset can dereference a missing model.** MotorIOSparkMax.java:117 and the
  corresponding Flex implementation access mechSim during encoder reset without checking
  whether a simulation was attached. Resetting an IO without a model can throw.
- **SPARK limit getters read limit-switch positions rather than software-limit
  thresholds.** The common MotorIO interface promises software limits; the implementations
  query configAccessor.limitSwitch instead of the configured soft-limit thresholds.
- **withRotorFeedback does not select rotor feedback on TalonFX.** That assignment is
  commented out. Starting from a remote-sensor config can leave the remote source active.
- **PhotonVision cached pose and uncertainty can come from different frames.** If a
  valid result is followed by an invalid result in one unread-results batch, latestPose
  remains the valid pose while curStdDevs is overwritten by the invalid frame's defaults.
- **Repeated tag measurements can be fused.** Current VisionHandler checks age but
  has no per-camera timestamp deduplication. A cached Limelight estimate may enter the
  estimator repeatedly during its freshness window. Quest frames have no explicit
  finite/freshness/reset-time filter or seed-confirmation gate.
- **GameState caches the auto winner indefinitely.** No mode transition resets it, so
  another match without restarting the robot can use the previous match's winner. It
  currently informs telemetry, not feeding policy.
- **Auto preload is permanently inhibited after the first autonomous.** autoHasRan is
  never reset. The PathPlanner auto can still reset drivetrain odometry on a later run,
  but the matching Quest reset from disabled preload will not occur. Test this in repeated
  practice autos without restarting the robot.
- **The control chooser is unfinished.** It is neither published nor read to update
  controlSet. overrideLobShot and manual raise/lower hopper controls are also unbound.

## Hardware configuration that needs verification

These findings identify code inconsistencies; physical wiring and encoder placement
are needed before choosing exact replacements.

- **Hood mixes motor and mechanism units.** Gear ratio is about 162.13:1. Motion speeds
  are divided by that ratio, but the controller sensor-to-mechanism ratio is commented
  out. Shot table values 5–20 are then wrapped in Rotations.of(), feedback is raw motor
  rotations, and tolerance is expressed as three degrees. If the table intentionally
  contains motor rotations, its upper value is physically plausible, but motion speed,
  tolerance, visualization, and API naming need the same convention. Do not blindly
  enable gearing without converting the table and gains. The hood has no configured
  software limits or explicit homing/absolute reference in the active configuration.
- **Hood motor identity differs from documentation/model.** Active IO creates a Kraken
  X60 TalonFX, simulation uses a Minion, and FOC notes describe a TalonFXS hood. Verify
  the actual motor/controller and correct all three representations.
- **Intake pivot fused feedback ratios look inconsistent with an output-shaft CANcoder.**
  SensorToMechanismRatio is 48 and RotorToSensorRatio is left at its default. If the
  CANcoder measures the mechanism directly, those should normally be 1 and 48 respectively.
  Verify the sensor location before changing them.
- **Intake roller gearing is omitted from real feedback.** Target speed is divided by
  three and simulation uses 3:1, but the active IO config does not set the feedback ratio.
  This commands the reduced number as rotor speed rather than output-shaft speed.
- **Intake roller current-limit constants are unused.** getIOConfig() applies gains,
  inversion, and braking, but not its declared supply/stator limits.
- **Kicker current-limit constants are also unused.** Its active config omits
  withCurrentLimits(), so the defined 50/120 numbers do not control the hardware.
- **Hopper PortConfig names the Right CANivore, but SparkMax construction accepts only
  the device ID and does not use kBus.** Confirm the actual bus wiring; the abstraction
  misleadingly suggests that this controller is selected on that CANivore.
- **PathPlanner's physical model differs from TunerConstants:** wheel radius 0.048 vs
  0.0508 m, gearing 5.143 vs 6.0268, max speed 5.45 vs 5.12 m/s, and module coordinates
  +/-0.273 m vs +/-0.257175 X and +/-0.295275 Y. Reconcile to measured hardware, including
  mass/MOI and autonomous current-limit assumptions, before trusting feedforwards.
- **Hopper gearing is explicitly a TODO.** Relative-encoder startup zero is not a
  homing procedure. Verify origin and travel before relying on its software limits.

## Maintainability and lower-priority utility issues

The common IO/request interfaces, typed quantities, separate constants, centralized
controls, subsystem command requirements, and supplier-driven RobotState are useful
choices. Follower construction retains controllers, clones configuration snapshots,
validates duplicate IDs, checks follower configuration results, and cleans up on failure.
That is a substantial improvement over unmanaged follower objects. Default roller stop
commands, both alliance-aware auto selection and pose flipping, build metadata, and
stable telemetry keys are also useful foundations.

The weaknesses are inconsistent lifecycle contracts and duplicated state:

- Most mechanism telemetry bodies are commented out. Only flywheel and power currently
  process their dedicated input records. Intake/hopper visualization caches never update.
- Replay is an advertised Mode value, but Robot never installs a replay source and the
  hardware/control paths read real wrappers directly. Replay is not an implemented workflow.
- Static Robot subsystem globals, public mutable shot fields, and reusable mutable motor
  requests make construction order and ownership harder to reason about. Inject suppliers
  where practical and return a typed ShotSolution instead of a double[3].
- Deactivate gates Commands.either at initialization, not continuously. Deactivation after
  a command starts does not guarantee that its output stops. Document or strengthen this
  contract before using it as an operational disable mechanism.
- Build.gradle declares AdvantageKit runtime/autolog 4.0.0 while the vendordep is 26.0.2.
  The current build resolves successfully, but align runtime and annotation processor
  versions deliberately rather than relying on dependency conflict resolution.
- There is no checked-in CI workflow, and all eight tests are confined to two drive helpers.
- Coordinate2d.getY() returns X. Vector2d uses atan(y/x) instead of atan2(y,x), losing
  quadrant information. These are library bugs rather than active shooting calculations.
- PiecewiseRegression extrapolates negative inputs between -delta and zero because an
  int cast truncates toward zero. It also lacks a positive-spacing/finite-input check.
  ShotTable builds a distances list but ignores it, assuming equally spaced samples.
- Regression's default inverse returns zero silently; singular linear/quadratic fits and
  constant-output residuals are not guarded. Prefer explicit unsupported errors and validation.
- Legacy Falcon-count conversion helpers coexist with modern rotation-based IO. Keep their
  scope clear to prevent an accidental unit conversion at a new call site.
- Several comments and docs describe older class names, topics, motor families, and settings.
  Configuration correctness matters more than comment volume; update docs with behavior.

## Previously discussed work: current status

Compared with this chat and the relevant earlier robotics chats:

| Work | Current status |
| --- | --- |
| Correct drive scalar order | Implemented; regression tests pass. |
| More responsive manual turning | Linear rotation curve and wheel-speed reservation implemented; physical feel unverified. |
| Preserve the commanded arc when desaturating | CTRE's default proportional desaturation remains, but the new limiter reduces translation first. That intentionally changes the requested arc under saturation. Reconcile this with the earlier explicit arc-preservation preference. |
| Correct Quest reset mounting transform | Present. |
| Verify AprilTag-derived Quest seeding | Earlier added handler logic and dashboard values are absent in the current handler. Constants and documentation remain. This is incomplete in the current tree. |
| Quest's own beta AprilTag detection | Separate from the robot's Limelight flag; headset configuration/status cannot be established from this Java code. |
| Robot-side Limelight AprilTags | Camera provider configured, but kUseAprilTags remains false. |
| Elastic selector and Oculus indicators | Layout exists; Oculus topics use SmartLogs/Vision while logger uses SmartLogs/VisionSubsystem. README also names different topics. Indicators need reconciliation. |
| USB logs organized by event and match | EventLogWriter is wired and supports testing/competition rotation. Still needs real USB/FMS validation. |
| BuildConstants generation | Wired before compile and working. |
| Power monitoring and simulated battery sag | Real telemetry and load registration are wired. Mechanism simulation lifecycle currently prevents trustworthy simulated load behavior. |
| CAN ID 22 follower conflict | Current tree assigns it only to the kicker, matching the earlier decision. |
| Earlier deliberate current-limit settings | Have changed: flywheel now 30 A supply/70 A stator vs earlier 30/60, indexer 40/120 vs earlier 40/70. Kicker previously requested 25/60; its current constants are unused. Verify intent and controller readbacks. |
| Optional FOC tuning baseline objects | Described by docs/foc-gain-tuning.md but absent from current constants. Characterization remains outstanding. |
| Continuous feeder gating during recovery | Still not implemented. |
| Input-lag/network-drop diagnosis | Not resolved by source review; repeated configuration writes are a credible candidate. Need DS and robot logs from the affected session. |
| Dedicated DriveCommand refactor | Scaffold only, not connected to the default drive behavior. |

The source-confirmed issues should be fixed before attributing remaining behavior to
PID alone. Then test autonomous command composition, heading wrap, repeated auto runs,
shoot/intake trigger transitions, recovery gating, simulated motor motion/current,
Quest disconnect/reset behavior, and dashboard topics. Finally characterize mechanisms
and compare requested versus actual speeds, voltage, current, and main-loop duration
on the robot under sustained passing.

Reference semantics checked against WPILib's command documentation:
https://docs.wpilib.org/en/stable/docs/software/commandbased/commands.html
and the locally cached Phoenix 26.3.0/AdvantageKit 26.0.2 sources. Phoenix's current
VelocityVoltage and DynamicMotionMagicVoltage defaults enable FOC; selecting a factory
named FOC is not itself the control-mode switch, and licensing/hardware still require
verification. No claim that the code disables FOC was made from factory names alone.
