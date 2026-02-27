package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import java.util.List;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "CompGroup2026", group = "Linear Opmode")
public class TeleopV6_2026 extends LinearOpMode {

    private Bumble robot;
    private final ShooterV1 shooter = new ShooterV1();
    private final distancestuff distanceReader = new distancestuff(); // only for telemetry display
    private final Trigger trigger = new Trigger();

    private DecodeObeliskVision decodeVision;
    private FtcDashboard dashboard;

    // ============================================================
    // Goal tag IDs you confirmed:
    //   BLUE goal = 20
    //   RED goal  = 24
    // ============================================================
    private static final int BLUE_GOAL_ID = 20;
    private static final int RED_GOAL_ID = 24;

    // Track both, then pick whichever is visible (your camera can't see both at once anyway)
    private final GoalTagTracker blueGoalTracker = new GoalTagTracker(BLUE_GOAL_ID);
    private final GoalTagTracker redGoalTracker = new GoalTagTracker(RED_GOAL_ID);

    // ===== Math/physics class that converts distance -> RPM =====
    private final ShooterRPMCalculator rpmCalc = new ShooterRPMCalculator();

    // ===== If no GOAL AprilTag distance is available, use this RPM =====
    private static final double DEFAULT_RPM_NO_DISTANCE = 87.0;

    // ===== LED "ready" tolerance in RPM (tighter than 8) =====
    private static final double READY_TOLERANCE_RPM = 3.0; // try 3.0; use 2.0 if stable

    // ===== Shooter toggle state (press trigger once to enable; A disables) =====
    private boolean shooterEnabled = false;

    // ===== Variables for smoothing & rate limiting =====
    private double filteredRpm = 0.0;
    private double lastLoopTimeSec = 0.0;

    // These control how smoothly RPM adapts
    private static final double RPM_SMOOTH_ALPHA = 0.25;   // smoothing factor (0..1)
    private static final double RPM_SLEW_PER_SEC = 180.0;  // max RPM change per second

    @Override
    public void runOpMode() {

        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(super.telemetry, dashboard.getTelemetry());

        robot = new Bumble(telemetry);

        // DecodeObeliskVision owns the camera + AprilTag processor and exposes detections.
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

        lastLoopTimeSec = getRuntime();
        double drivePower = -0.3;

        while (opModeIsActive()) {

            blinkin.setIdle();

            // ============================================================
            // ====================== DRIVE CONTROL =======================
            // ============================================================
            if (gamepad1.left_trigger > 0) drivePower = -0.2;
            else if (gamepad1.right_trigger > 0) drivePower = -0.8;
            else drivePower = -0.6;

            if (gamepad1.dpad_up) robot.driveForward(drivePower);
            else if (gamepad1.dpad_down) robot.driveBackward(drivePower);
            else if (gamepad1.dpad_left) robot.strafeLeft(drivePower);
            else if (gamepad1.dpad_right) robot.strafeRight(drivePower);
            else if (gamepad1.right_bumper) robot.rotateLeft(drivePower);
            else if (gamepad1.left_bumper) robot.rotateRight(drivePower);
            else robot.allStop();

            // ============================================================
            // ================= SHOOTER TOGGLE CONTROL ===================
            // ============================================================
            // Press RIGHT trigger once (or hold briefly) to enable shooter.
            // A turns shooter OFF.
            if (gamepad2.right_trigger > 0.1) {
                shooterEnabled = true;
            }
            if (gamepad2.left_trigger > 0.1) {
                //shooter.shoot(0.40);
                shooter.setTargetRpm(87); // “high”
                //shooter.setTargetRpm(200); // “high”
            }
            if (gamepad2.a) {
                    shooterEnabled = false;
                    shooter.stop();
                    filteredRpm = 0.0;
            }
            if (gamepad2.b) {
                trigger.fire(this);
            }

                // ============================================================
                // =========== UPDATE VISION + TRACK GOAL TAG =================
                // ============================================================
                GoalTagTracker activeGoal = null;
                Double goalRangeInches = null;
                Double goalBearingDeg = null;
                int goalId = -1;

                if (decodeVision != null) {
                    // Refresh detections
                    decodeVision.update();

                    // Share detections with both goal trackers
                    List<AprilTagDetection> dets = decodeVision.getDetections();
                    blueGoalTracker.update(dets);
                    redGoalTracker.update(dets);

                    // Pick whichever goal is visible (camera can't see both at once)
                    if (blueGoalTracker.isGoalVisible()) activeGoal = blueGoalTracker;
                    else if (redGoalTracker.isGoalVisible()) activeGoal = redGoalTracker;

                    // Pull range/bearing from the active goal
                    if (activeGoal != null) {
                        goalId = activeGoal.getGoalId();
                        goalRangeInches = activeGoal.getRangeInches();
                        goalBearingDeg = activeGoal.getBearingDeg();
                    }

                    // ================= GOAL TELEMETRY =================
                    telemetry.addLine("=== GOAL TAG (AUTO RED/BLUE) ===");
                    telemetry.addData("Goal Visible", activeGoal != null);
                    telemetry.addData("Goal ID", (activeGoal != null) ? goalId : "—");
                    telemetry.addData("Goal Range (in)", (goalRangeInches != null) ? String.format("%.1f", goalRangeInches) : "—");
                    telemetry.addData("Goal Bearing (deg)", (goalBearingDeg != null) ? String.format("%.1f", goalBearingDeg) : "—");
                }

                // ============================================================
                // ================= ADAPTIVE SHOOTER RPM =====================
                // ============================================================
                if (shooterEnabled) {
                    // Use GOAL distance for RPM math. If goal not visible or no pose -> fallback to 87.
                    Double rangeInches = goalRangeInches;

                    double desiredRpm;
                    if (rangeInches == null || Double.isNaN(rangeInches) || rangeInches <= 0) {
                        desiredRpm = DEFAULT_RPM_NO_DISTANCE;
                    } else {
                        desiredRpm = rpmCalc.getTargetRpmFromCameraToTagInches(rangeInches, DEFAULT_RPM_NO_DISTANCE);
                    }

                    // Smooth + rate limit so RPM adapts continuously without hunting
                    double now = getRuntime();
                    double dt = now - lastLoopTimeSec;
                    if (dt <= 0) dt = 0.02;

                    // Exponential smoothing on RPM setpoint
                    if (filteredRpm <= 0) {
                        filteredRpm = desiredRpm;
                    } else {
                        filteredRpm = (RPM_SMOOTH_ALPHA * desiredRpm) + ((1.0 - RPM_SMOOTH_ALPHA) * filteredRpm);
                    }

                    // Slew-limit RPM changes
                    double maxDelta = RPM_SLEW_PER_SEC * dt;
                    double delta = filteredRpm - shooter.getTargetRpm();
                    if (delta > maxDelta) filteredRpm = shooter.getTargetRpm() + maxDelta;
                    if (delta < -maxDelta) filteredRpm = shooter.getTargetRpm() - maxDelta;

                    // Send final RPM to shooter
                    shooter.setTargetRpm(filteredRpm);

                    lastLoopTimeSec = now;
                }

                // ============================================================
                // ====================== TRIGGER LOGIC =======================
                // ============================================================
                // B fires only if shooter is near target speed (tight tolerance)
                if (gamepad2.b && shooterEnabled && shooter.isAtSpeed(READY_TOLERANCE_RPM)) {
                    trigger.fire(this);
                }

                // ============================================================
                // ================= LED STATUS (PINK/YELLOW/GREEN) ============
                // ============================================================
                // Pink default while OpMode active if shooter is OFF.
                // Yellow while spinning up.
                // Green when ready to shoot (tight tolerance).
                double tgt = shooter.getTargetRpm();

                if (!shooterEnabled || tgt <= 0) {
                    blinkin.setIdle();          // PINK (your setIdle() is pink)
                } else if (!shooter.isAtSpeed(READY_TOLERANCE_RPM)) {
                    blinkin.setSpinning();      // YELLOW
                } else {
                    blinkin.setReady();         // GREEN
                }

                // ============================================================
                // ======================== TELEMETRY =========================
                // ============================================================
                telemetry.addData("Shooter Enabled", shooterEnabled);
                telemetry.addData("Target RPM", "%.1f", shooter.getTargetRpm());
                telemetry.addData("Shooter RPM", "%.1f", shooter.getAverageRpm());

                double inches = distanceReader.getDistanceInches();
                telemetry.addData("DistanceSensor (in)", Double.isNaN(inches) ? "—" : String.format("%.1f", inches));

                telemetry.update();
            }

            shooter.stop();
            robot.allStop();

            if (decodeVision != null) decodeVision.close();
            if (dashboard != null) dashboard.stopCameraStream();
        }
    }