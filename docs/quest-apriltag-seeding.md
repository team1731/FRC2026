# QuestNav AprilTag seeding

Enable `VisionConstants.kUseAprilTags` to use the configured `limelight-main` camera.
Check the camera name, field layout, camera mounting transform, and drivetrain heading
on the actual robot before using autonomous. AprilTags remain disabled by default.

While disabled, the robot must remain below 0.05 m/s and 5 degrees/s for 0.5 seconds.
A fresh accepted AprilTag measurement then supplies the robot translation for a Quest
pose reset. The reset retains the drivetrain heading and applies the robot-to-Quest
mounting transform. An unconfirmed request can retry after two seconds while disabled.
Autonomous start-pose resets clear the AprilTag seed status.

Dashboard values (also recorded under SmartLogs/VisionSubsystem):

- `Vision/Quest Seeded From AprilTags`: an AprilTag-derived reset was requested;
  this alone does not prove the headset applied it.
- `Vision/Quest Seed Verified`: five distinct fresh tag frames agree with post-reset
  Quest robot translation within 0.10 m while stationary. Frames must be at most
  0.25 seconds old and their timestamps within 0.10 seconds of the Quest frame.
- `Vision/Quest Seed Confirmed`: verification succeeded for this seed. This stays
  true during motion or loss of tag visibility so Quest odometry can continue.
  A pose reset, Quest tracking/connection loss, or a stationary disagreement clears it.
- `Vision/Quest Seed Position Error Meters`: the latest comparison error (NaN until
  a comparison is available).
- `Vision/Quest Seed Last Verification Time`: last successful verification in FPGA
  seconds (negative infinity until verified).

The live Verified value turns false during motion or when comparisons become stale.
With AprilTags enabled, Quest measurements enter drivetrain estimation only after
confirmation. Camera measurements still enter the estimator independently.
Verification checks translation only: MegaTag2 uses the supplied drivetrain heading,
and the configured tag measurement heading uncertainty is deliberately large.

On hardware, test with the robot disabled and stationary facing tags: Confirmed and
Verified should turn true. Drive and confirm Verified clears while Confirmed remains
true. Cover the tag camera and confirm Verified expires. Disconnect/restart Quest and
confirm Confirmed clears, then returns after reseeding with tags visible. A known
wrong robot heading must be corrected separately; the translation indicator cannot
validate it. Tune the tolerances in VisionConstants using recorded position errors.
