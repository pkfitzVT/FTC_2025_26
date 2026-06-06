package org.firstinspires.ftc.teamcode;

/**
 * ShotCalculator
 * -------------
 * A clean, hardware-free "math only" class that converts a measured distance-to-goal
 * into a recommended shooter RPM using ideal projectile motion.
 *
 * WHY THIS CLASS EXISTS:
 * - ShooterV1 should ONLY control motors (setTargetRpm, read RPM, isAtSpeed).
 * - TeleOp should orchestrate: read sensors -> ask ShotCalculator -> command ShooterV1.
 * - This class contains all the "physics + calibration" in one place, so it is easy to tune.
 *
 * WHAT IT COMPUTES:
 * Given:
 *   - launch height y0 (inches)
 *   - launch angle theta (degrees)
 *   - desired target height yT (inches) at the goal plane (e.g., "center of goal")
 *   - horizontal distance x (inches) from launcher to goal plane
 *   - gravity g (in/s^2)
 * It computes the minimum launch speed v0 (in/s) needed so that the projectile
 * passes through (x, yT).
 *
 * Then it converts v0 to an RPM command using an empirically measured scale:
 *   v0 ≈ k * RPM   =>   RPM ≈ v0 / k
 *
 * IMPORTANT REAL-WORLD NOTES:
 * - This is an IDEAL model (no air drag, no wobble, no slip).
 * - In practice you will likely add a small offset/trim and/or clamp RPM.
 * - Your launch angle may drift slightly with RPM; this model assumes constant angle.
 */
public class ShotCalculator {

    // ----------------------------
    // Constants you can tune
    // ----------------------------

    // Gravity in inches/sec^2. (32.174 ft/s^2 * 12 in/ft ≈ 386.1 in/s^2)
    private static final double G_IN_PER_S2 = 386.1;

    // Your geometry assumptions:
    private final double launchHeightIn;      // y0
    private final double launchAngleDeg;      // theta in degrees
    private final double targetHeightIn;      // yT (center height you want at the goal plane)

    // If your AprilTag range is NOT exactly to the goal "impact plane", you can correct it.
    // You said: "horizontal distance is 6 inches longer than distance to the AprilTag".
    private final double xOffsetIn;           // x = rangeToTag + xOffsetIn

    // Empirical speed-to-RPM calibration:
    // You measured:
    //   85 RPM -> 176 in/s  => k ≈ 2.07 in/s per RPM
    //   127 RPM -> 265 in/s => k ≈ 2.09 in/s per RPM
    // A good starting average is about 2.08.
    private final double inchesPerSecondPerRpm; // k

    // Safety clamps so we never command crazy RPM due to a bad sensor read.
    private final double minRpm;
    private final double maxRpm;

    // Optional constant "bias" RPM to account for drag / real-world losses
    // (Start at 0. Increase if shots consistently fall short.)
    private final double rpmBias;

    // ----------------------------
    // Constructor
    // ----------------------------

    /**
     * @param launchHeightIn           y0 (inches), e.g. 14
     * @param launchAngleDeg           theta (degrees), e.g. 71
     * @param targetHeightIn           yT (inches), e.g. 41 (center of goal plane)
     * @param xOffsetIn                add to measured range (inches), e.g. +6
     * @param inchesPerSecondPerRpm    k (in/s per RPM), e.g. 2.08
     * @param minRpm                   clamp minimum RPM (e.g. 0 or 60)
     * @param maxRpm                   clamp maximum RPM (e.g. 140 indoor cap or higher in match)
     * @param rpmBias                  small constant add-on to fight drag (start 0)
     */
    public ShotCalculator(double launchHeightIn,
                          double launchAngleDeg,
                          double targetHeightIn,
                          double xOffsetIn,
                          double inchesPerSecondPerRpm,
                          double minRpm,
                          double maxRpm,
                          double rpmBias) {

        this.launchHeightIn = launchHeightIn;
        this.launchAngleDeg = launchAngleDeg;
        this.targetHeightIn = targetHeightIn;
        this.xOffsetIn = xOffsetIn;
        this.inchesPerSecondPerRpm = inchesPerSecondPerRpm;

        this.minRpm = minRpm;
        this.maxRpm = maxRpm;
        this.rpmBias = rpmBias;
    }

    // ----------------------------
    // Public API
    // ----------------------------

    /**
     * Compute the recommended RPM based on a measured distance (typically AprilTag range).
     *
     * @param rangeToTagIn  measured range to AprilTag (inches).
     *                     If tag is unavailable, you can pass a distance sensor range instead.
     * @return rpm command (clamped). Returns 0 if the math says the shot is impossible
     *         with the given angle/geometry (or if input is invalid).
     */
    public double rpmFromRange(double rangeToTagIn) {

        // Basic input validation
        if (Double.isNaN(rangeToTagIn) || rangeToTagIn <= 0) {
            return 0;
        }

        // Convert sensor range to the horizontal distance we use in the projectile equation.
        // You stated: "horizontal distance is 6 inches longer than the distance to the april tag".
        double x = rangeToTagIn + xOffsetIn;

        // Compute required launch speed v0 (in/s) for the projectile to pass through (x, targetHeightIn).
        double v0 = requiredLaunchSpeedForPoint(x, targetHeightIn);

        if (v0 <= 0) {
            // Means "impossible" with these assumptions (or numerical issue).
            return 0;
        }

        // Convert speed -> RPM using empirical calibration.
        // v ≈ k * RPM  => RPM ≈ v/k
        double rpm = (v0 / inchesPerSecondPerRpm) + rpmBias;

        // Clamp for safety and practicality.
        return clamp(rpm, minRpm, maxRpm);
    }

    /**
     * Optional helper: get the predicted launch speed (in/s) for debugging / telemetry.
     */
    public double speedFromRange(double rangeToTagIn) {
        if (Double.isNaN(rangeToTagIn) || rangeToTagIn <= 0) return 0;
        double x = rangeToTagIn + xOffsetIn;
        double v0 = requiredLaunchSpeedForPoint(x, targetHeightIn);
        return Math.max(0, v0);
    }

    // ----------------------------
    // Core Physics
    // ----------------------------

    /**
     * Physics derivation (ideal projectile, no drag):
     *
     * Let:
     *   y0 = launchHeightIn
     *   theta = launchAngleDeg
     *   g = G_IN_PER_S2
     *   v0 = initial speed
     *
     * Decompose velocity:
     *   v0x = v0 * cos(theta)
     *   v0y = v0 * sin(theta)
     *
     * Equations of motion:
     *   x(t) = v0x * t
     *   y(t) = y0 + v0y * t - (1/2) g t^2
     *
     * Solve for time at a given horizontal distance x:
     *   t = x / v0x = x / (v0 cos(theta))
     *
     * Substitute into y(t):
     *   y(x) = y0 + (v0 sin(theta))*(x/(v0 cos(theta))) - (1/2)g*(x^2/(v0^2 cos^2(theta)))
     *
     * Simplify sin/cos:
     *   y(x) = y0 + x tan(theta) - (g x^2) / (2 v0^2 cos^2(theta))
     *
     * We want y(x) = yTarget. Solve for v0:
     *   yTarget = y0 + x tan(theta) - (g x^2) / (2 v0^2 cos^2(theta))
     *
     * Rearranged:
     *   (g x^2) / (2 v0^2 cos^2(theta)) = y0 + x tan(theta) - yTarget
     *
     * Therefore:
     *   v0^2 = (g x^2) / (2 cos^2(theta) * (y0 + x tan(theta) - yTarget))
     *
     * Constraint:
     *   (y0 + x tan(theta) - yTarget) must be > 0, otherwise no real solution
     *   (it would require imaginary v0, meaning the line at that angle cannot hit that point).
     */
    private double requiredLaunchSpeedForPoint(double x, double yTarget) {

        double thetaRad = Math.toRadians(launchAngleDeg);

        double cos = Math.cos(thetaRad);
        double tan = Math.tan(thetaRad);

        // Denominator term: (y0 + x*tan(theta) - yTarget)
        // This is basically "how much vertical room the line-of-angle gives you"
        // before gravity pulls you down to yTarget.
        double denomRoom = launchHeightIn + (x * tan) - yTarget;

        // If denomRoom <= 0, then with this fixed angle, the projectile cannot pass through that point.
        // Example: If yTarget is too high at that distance for the chosen launch angle.
        if (denomRoom <= 0) {
            return 0;
        }

        // v0^2 = (g x^2) / (2 cos^2(theta) * denomRoom)
        double v0Squared = (G_IN_PER_S2 * x * x) / (2.0 * cos * cos * denomRoom);

        if (v0Squared <= 0 || Double.isNaN(v0Squared) || Double.isInfinite(v0Squared)) {
            return 0;
        }

        return Math.sqrt(v0Squared);
    }

    // ----------------------------
    // Utility
    // ----------------------------

    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}