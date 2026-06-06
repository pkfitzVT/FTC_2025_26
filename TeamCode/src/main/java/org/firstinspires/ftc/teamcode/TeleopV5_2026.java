package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.DecodeObeliskVision.Motif;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

//@TeleOp(name = "CompGroup_tele_2026", group = "Linear Opmode")
public class TeleopV5_2026 extends LinearOpMode {

    private Bumble robot;
    private final ShooterV1 shooter = new ShooterV1();
    private final distancestuff distanceReader = new distancestuff();
    private final Trigger trigger = new Trigger();

    private DecodeObeliskVision decodeVision;
    private FtcDashboard dashboard;

    private final GoalTagTracker blueGoalTracker = new GoalTagTracker(20);

    // ===== AUTO RPM FROM APRILTAG RANGE (Projectile Model) =====
    // Match your simulator assumptions (tune if needed)
    private static final double LAUNCH_ANGLE_DEG = 71.0;
    private static final double H0_IN = 14.0;
    private static final double LIP_H_IN = 39.0;
    private static final double G_IN_S2 = 386.09;

    // Your calibration: (87 rpm -> 176 in/s), (127 rpm -> 264 in/s)
    // v = A*rpm + B
    // A = (264-176)/(127-87) = 88/40 = 2.2
    // B = 176 - 2.2*87 = -15.4
    private static final double RPM_TO_V_A = 2.2;
    private static final double RPM_TO_V_B = -15.4;

    // Convert tag-measured range to "distance along shot to lip plane"
    // Tune this on-field. If the tag is BEHIND the lip by 6", set +6.
    private static final double TAG_RANGE_TO_LIP_OFFSET_IN = 4.0;

    // Add margin for drag / losses
    private static final double RPM_MARGIN = 3.0;

    // Only auto-rpm if bearing is near center
    private static final double BEARING_OK_DEG = 12.0;

    // Safety clamps
    private static final double RPM_MIN = 60.0;
    private static final double RPM_MAX = 140.0;

    // Default / fallback RPM if you want one
    private static final double RPM_DEFAULT = 87.0;

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static double requiredVelocityForLipAtDistance(double xIn) {
        double theta = Math.toRadians(LAUNCH_ANGLE_DEG);
        double ct = Math.cos(theta);
        double tt = Math.tan(theta);

        // v^2 = g x^2 / (2 cos^2(theta) (h0 + x tan(theta) - h_lip))
        double denom = 2.0 * ct * ct * (H0_IN + xIn * tt - LIP_H_IN);
        if (denom <= 0) return Double.NaN;

        return Math.sqrt((G_IN_S2 * xIn * xIn) / denom);
    }

    private static double rpmFromVelocity(double vInPerSec) {
        return (vInPerSec - RPM_TO_V_B) / RPM_TO_V_A;
    }

    private static double rpmForTagRange(double tagRangeIn) {
        double xLip = tagRangeIn + TAG_RANGE_TO_LIP_OFFSET_IN;
        if (xLip < 1.0) xLip = 1.0;

        double vNeed = requiredVelocityForLipAtDistance(xLip);
        if (!Double.isFinite(vNeed) || vNeed <= 0) return Double.NaN;

        double rpm = rpmFromVelocity(vNeed) + RPM_MARGIN;
        return clamp(rpm, RPM_MIN, RPM_MAX);
    }

    @Override
    public void runOpMode() {
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(super.telemetry, dashboard.getTelemetry());

        robot = new Bumble(telemetry);

        decodeVision = new DecodeObeliskVision(telemetry);
        decodeVision.init(hardwareMap);

        dashboard.startCameraStream(decodeVision.getVisionPortal(), 30);

        BlinkinSubsystem blinkin = new BlinkinSubsystem();

        robot.init(hardwareMap);
        robot.resetEncoders();
        robot.setRunWithoutEncoders();

        shooter.init(hardwareMap);
        trigger.init(hardwareMap);
        distanceReader.init(hardwareMap, "distance_sensor");
        blinkin.init(hardwareMap, "led");

        waitForStart();
        if (isStopRequested()) return;

        double drivePower = -0.3;

        // We keep the last commanded target so we can HOLD it when pose is missing
        double lastTargetRpm = RPM_DEFAULT;

        while (opModeIsActive()) {

            // =========================
            // 1) UPDATE VISION FIRST
            // =========================
            if (decodeVision != null) {
                decodeVision.update();
                blueGoalTracker.update(decodeVision.getDetections());
            }

            // Read goal pose AFTER updating
            Double gr = blueGoalTracker.getRangeInches();    // inches
            Double gb = blueGoalTracker.getBearingDeg();     // degrees
            boolean havePose = (gr != null && gb != null);

            // =========================
            // 2) DRIVE (gamepad1)
            // =========================
            if (gamepad1.left_trigger > 0) {
                drivePower = -0.2;
            } else if (gamepad1.right_trigger > 0) {
                drivePower = -0.8;
            } else {
                drivePower = -0.6;
            }

            if (gamepad1.dpad_up) {
                robot.driveForward(drivePower);
            } else if (gamepad1.dpad_down) {
                robot.driveBackward(drivePower);
            } else if (gamepad1.dpad_left) {
                robot.strafeLeft(drivePower);
            } else if (gamepad1.dpad_right) {
                robot.strafeRight(drivePower);
            } else if (gamepad1.right_bumper) {
                if (drivePower < 0) robot.rotateLeft(drivePower);
                else                 robot.rotateRight(drivePower);
            } else if (gamepad1.left_bumper) {
                if (drivePower < 0) robot.rotateRight(drivePower);
                else                 robot.rotateLeft(drivePower);
            } else {
                robot.allStop();
            }

            // =========================
            // 3) SHOOTER AUTO RPM (gamepad2)
            // =========================
            // ===== Continuous AUTO RPM while shooter is ON (RT held) =====
            if (gamepad2.right_trigger > 0.1) {

                boolean aimedOk  = havePose && Math.abs(gb) <= BEARING_OK_DEG;

                if (aimedOk) {
                    double rpmAuto = rpmForTagRange(gr); // gr is Double, auto-unboxes to double
                    if (Double.isFinite(rpmAuto)) {
                        lastTargetRpm = rpmAuto;                 // HOLD last good RPM
                        shooter.setTargetRpm(lastTargetRpm);     // updates every loop
                        telemetry.addData("AUTO RPM", "%.1f", rpmAuto);
                    } else {
                        shooter.setTargetRpm(RPM_DEFAULT);
                        telemetry.addLine("AUTO RPM NaN -> fallback 87");
                    }
                } else {
                    // Hold lastTargetRpm instead of snapping to 87
                    shooter.setTargetRpm(lastTargetRpm);
                    telemetry.addLine("AUTO RPM: no pose / not aimed -> HOLD");
                    if (havePose) telemetry.addData("Bearing", "%.1f", gb);
                }

            } else if (gamepad2.left_trigger > 0.1) {
                lastTargetRpm = 75;
                shooter.setTargetRpm(lastTargetRpm);
            } else if (gamepad2.a) {
                shooter.stop();
            } else if (gamepad2.left_trigger > 0.1) {
                shooter.setTargetRpm(75);
            } else if (gamepad2.a) {
                shooter.stop();
            }

            // =========================
            // 4) TRIGGER
            // =========================
            if (gamepad2.b && shooter.isAtSpeed(2)) {
                trigger.fire(this);
            }

            // =========================
            // 5) TELEMETRY
            // =========================
            telemetry.addLine("\n=== BLUE GOAL TAG ===");
            telemetry.addData("Goal Visible", blueGoalTracker.isGoalVisible());
            telemetry.addData("Goal ID", blueGoalTracker.getGoalId());
            telemetry.addData("Goal Range (in)", havePose ? String.format("%.1f", gr) : "—");
            telemetry.addData("Goal Bearing (deg)", havePose ? String.format("%.1f", gb) : "—");

            telemetry.addData("Shooter RPM Avg", "%.1f", shooter.getAverageRpm());
            telemetry.addData("Shooter Target RPM", "%.1f", shooter.getTargetRpm());

            telemetry.update();
        }

        shooter.stop();
        robot.allStop();

        if (decodeVision != null) decodeVision.close();
        if (dashboard != null) dashboard.stopCameraStream();
    }
}