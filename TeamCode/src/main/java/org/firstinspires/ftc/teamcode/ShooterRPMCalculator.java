package org.firstinspires.ftc.teamcode;

/**
 * ShooterRPMCalculator
 *
 * This class performs ALL math needed to convert:
 *
 *   AprilTag distance  ->  required projectile velocity  ->  flywheel RPM
 *
 * It contains ONLY physics/math — no hardware control.
 */
public class ShooterRPMCalculator {

    // ===================== GEOMETRY CONSTANTS =====================

    private static final double ARTIFACT_DIAMETER = 5.0;     // inches
    private static final double LAUNCH_BOTTOM = 14.0;        // bottom height at launch
    private static final double WALL_HEIGHT = 39.0;          // wall height
    private static final double AFTER_WALL = 6.25;            // target point after wall

    // Convert bottom heights to CENTER heights for projectile math
    private static final double Y0 = LAUNCH_BOTTOM + ARTIFACT_DIAMETER / 2.0;      // 16.5 in
    private static final double Y_TARGET = WALL_HEIGHT + ARTIFACT_DIAMETER / 2.0;  // 41.5 in

    // Launch angle (fixed)
    private static final double THETA_DEG = 71.0;

    // Gravity (inches/sec^2)
    private static final double G = 386.09;

    // Velocity -> RPM conversion (from your calibration)
    private static final double RPM_SLOPE = 0.512821;
    private static final double RPM_INTERCEPT = -8.89744;

    // Safety limits
    private static final double MIN_RPM = 0;
    private static final double MAX_RPM = 250;

    /**
     * Main function called by TeleOp
     */
    public double getTargetRpmFromCameraToTagInches(double cameraToTagInches, double defaultRpm) {

        if (Double.isNaN(cameraToTagInches) || cameraToTagInches <= 0)
            return defaultRpm;

        // Horizontal distance from shooter to desired height point
        double x = cameraToTagInches + AFTER_WALL;

        // Compute required projectile speed using physics
        double velocity = computeRequiredVelocity(x, Y0, Y_TARGET);

        if (Double.isNaN(velocity) || velocity <= 0)
            return defaultRpm;

        // Convert velocity to RPM
        double rpm = RPM_SLOPE * velocity + RPM_INTERCEPT;

        // Clamp RPM into safe range
        if (rpm < MIN_RPM) rpm = MIN_RPM;
        if (rpm > MAX_RPM) rpm = MAX_RPM;

        return rpm;
    }

    /**
     * Solves projectile equation for required launch velocity.
     *
     * y = y0 + x*tan(theta) - (g*x^2)/(2*v^2*cos^2(theta))
     *
     * Rearranged to solve for v.
     */
    private double computeRequiredVelocity(double x, double y0, double yTarget) {

        double theta = Math.toRadians(THETA_DEG);
        double cos = Math.cos(theta);
        double tan = Math.tan(theta);

        double deltaY = yTarget - y0;

        double denominator = (x * tan) - deltaY;

        // If <= 0, trajectory is impossible
        if (denominator <= 0) return Double.NaN;

        double numerator = G * x * x;
        double bottom = 2 * cos * cos * denominator;

        if (bottom <= 0) return Double.NaN;

        return Math.sqrt(numerator / bottom);
    }
}