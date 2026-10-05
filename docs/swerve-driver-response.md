# Driver scaling and turning response

DriveScalar takes a normalized joystick input, clamps it to [-1, 1], applies a
rescaled deadband, shapes the curve, optionally slew-limits the normalized output,
and finally multiplies by its configured scalar. Use `withScalar(maxSpeed)` and
pass the joystick value directly to `scale`; never multiply by speed before shaping.

Translation uses a quadratic curve and rotation uses a linear curve. Speed factors
are configured in SwerveConstants. The existing 0.6 rotation factor now applies
once, giving a maximum driver rotation request of 5.65 rad/s (324 degrees/s).
The joystick CTRE requests have no second deadband. Snail mode applies afterward.
Target-lock translation uses the same corrected scaling with its half-speed factor.

Previously a full forward input was squared after multiplication by 5.12 m/s,
producing a request of 26.21 rather than 5.12 m/s. Full rotation similarly requested
31.98 rather than 5.65 rad/s. These excessive combined requests cannot be achieved
by the modules and distort the relationship between stick input and robot motion.

Normal driving and joystick target lock now reserve wheel speed for rotation:

`translation speed <= max wheel speed - abs(rotation rate) * drive radius`

This conservative bound is independent of heading and alliance perspective. It
preserves requested rotation whenever rotation alone is feasible, reducing
translation while keeping its direction. Diagonal translation is also limited to
the available speed. CTRE's wheel-speed desaturation remains enabled as a fallback.
At full forward/full rotation, expect translation to fall to about 2.92 m/s so the
modules have room to turn. This deliberately trades forward speed for turn response.
It does not change autonomous path-following requests, motor gains, or current limits.

Driver Requested Speeds and Driver Limited Speeds are logged under the swerve
SmartLogs folder; both use driver field coordinates. Current Speeds is robot-relative,
so compare their omega fields directly and their translation magnitudes rather than
individual X/Y fields.

On the robot, compare turning from rest and at full forward stick, then test diagonals,
small stick inputs just above deadband, and snail mode. Requested and limited omega
should match during ordinary driver turns. If measured omega still lags the limited
request, examine battery voltage, drive current limiting, wheel slip, and module
steering tracking in logs before tuning gains. Automated tests cover scalar order,
deadband continuity, direction preservation, and module speed bounds across headings.
