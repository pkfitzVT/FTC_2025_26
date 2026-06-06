package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// Dashboard streaming
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

/**
 * CompGroup2026_AutoRPM_TOF_Latched
 * --------------------------------
 * Uses ONLY the REV 2m distance sensor (ToF) as the distance source.
 *
 * Operator controls (gamepad2):
 *  - RIGHT TRIGGER: turns shooter ON (latched). You can release it; shooter stays on.
 *  - B: turns shooter OFF (latched off).
 *  - RIGHT BUMPER: FIRE (launch artifact) -- only fires if shooter is READY.
 *
 * LED (Blinkin):
 *  - Pink   = shooter OFF / idle
 *  - Yellow = shooter ON but not at speed yet (spinning)
 *  - Green  = shooter READY (at speed for several consecutive loops)
 *
 * Sequence each loop:
 *  1) Read ToF distance (inches)
 *  2) Validate distance
 *  3) Compute ideal RPM from distance using ShotCalculator (adds +6 inches internally)
 *  4) If shooter is latched ON, command that RPM continuously
 *  5) Determine READY using tight RPM tolerance + stable loop count
 *  6) LED indicates state; FIRE only if READY
 *
 * Requires these classes already in your project:
 *  - ShotCalculator.java
 *  - BlinkinSubsystem.java (RevBlinkinLedDriver-based; name "led")
 */
//@TeleOp(name = "CompGroup2026_AutoRPM_TOF_Latched", group = "Linear Opmode")
public class TeleopV3_2025_AutoRPM extends LinearOpMode {

    private Bumble robot;

    private final ShooterV1 shooter = new ShooterV1();
    private final Trigger trigger = new Trigger();
    private final distancestuff distanceReader = new distancestuff();

    private FtcDashboard dashboard;

    // LED subsystem (RevBlinkinLedDriver-based; hardware name "led")
    private final BlinkinSubsystem blinkin = new BlinkinSubsystem();

    // Physics calculator (pure math)
    private ShotCalculator shotCalc;

    // --- Tunables ---
    // You preferred tight tolerance. If READY almost never happens, loosen to 3–4.
    private static final double RPM_TOLERANCE = 2.0;

    // Require N consecutive loops at-speed before we declare READY.
    private static final int READY_STABLE_LOOPS = 6;

    // RPM clamp safety (adjust for indoor ceiling vs match)
    private static final double MIN_RPM = 60;
    private static final double MAX_RPM = 140;

    // Distance sanity checks for ToF
    private static final double MIN_VALID_RANGE_IN = 10;
    private static final double MAX_VALID_RANGE_IN = 140;

    // --- State ---
    private boolean shooterLatchedOn = false;

    private boolean prevRtOn = false;      // edge detect for RT "turn on"
    private boolean prevOffB = false;      // edge detect for B "turn off"
    private boolean prevFire = false;      // edge detect for fire button

    private int readyCount = 0;

    @Override
    public void runOpMode() {

        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(super.telemetry, dashboard.getTelemetry());

        robot = new Bumble(telemetry);
        robot.init(hardwareMap);
        robot.resetEncoders();
        robot.setRunWithoutEncoders();

        shooter.init(hardwareMap);
        trigger.init(hardwareMap);

        distanceReader.init(hardwareMap, "distance_sensor");
        blinkin.init(hardwareMap, "led");

        // Shot calculator constants (your setup)
        shotCalc = new ShotCalculator(
                14.0,   // launchHeightIn
                71.0,   // launchAngleDeg
                41.0,   // targetHeightIn (center of goal plane)
                6.0,    // xOffsetIn (goal plane ~6" beyond measured distance)
                2.08,   // inchesPerSecondPerRpm (your calibration)
                MIN_RPM,
                MAX_RPM,
                0.0     // rpmBias (tune later if needed)
        );

        telemetry.addLine("AUTO RPM (ToF) - Latched Shooter");
        telemetry.addLine("RT = shooter ON (latch) | B = shooter OFF | RB = FIRE (READY only)");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        double drivePower = -0.3;

        while (opModeIsActive()) {

            // --------------------------
            // DRIVE (unchanged from your TeleOp)
            // --------------------------
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

            // ---------------------------------------------------
            // SHOOTER LATCH CONTROLS (NO helper methods in TeleOp)
            //
            // RT (rising edge) -> shooterLatchedOn = true
            // B  (rising edge) -> shooterLatchedOn = false
            // ---------------------------------------------------
            boolean rtNow = (gamepad2.right_trigger > 0.1);
            boolean rtEdgeOn = rtNow && !prevRtOn;
            prevRtOn = rtNow;

            if (rtEdgeOn) {
                shooterLatchedOn = true;
            }

            boolean offNow = gamepad2.b;
            boolean offEdge = offNow && !prevOffB;
            prevOffB = offNow;

            if (offEdge) {
                shooterLatchedOn = false;
            }

            // --------------------------
            // 1) Read ToF distance
            // --------------------------
            double tofRange = distanceReader.getDistanceInches();

            boolean tofValid = (!Double.isNaN(tofRange)
                    && tofRange >= MIN_VALID_RANGE_IN
                    && tofRange <= MAX_VALID_RANGE_IN);

            // --------------------------
            // 2) Compute ideal RPM (if distance valid)
            // --------------------------
            double rpmIdeal = tofValid ? shotCalc.rpmFromRange(tofRange) : 0;

            // --------------------------
            // 3) Command shooter (if latched on)
            // --------------------------
            boolean readyStable = false;

            if (!shooterLatchedOn || rpmIdeal <= 0) {
                shooter.stop();
                readyCount = 0;
                blinkin.setIdle(); // Pink
            } else {
                shooter.setTargetRpm(rpmIdeal);

                boolean atSpeedNow = shooter.isAtSpeed(RPM_TOLERANCE);
                if (atSpeedNow) readyCount++;
                else readyCount = 0;

                readyStable = (readyCount >= READY_STABLE_LOOPS);

                if (readyStable) blinkin.setReady();     // Green
                else             blinkin.setSpinning();  // Yellow
            }

            // ---------------------------------------------------
            // FIRE CONTROL (moved off of B, because B is OFF now)
            //
            // Here: RIGHT BUMPER fires on rising edge, ONLY if readyStable
            // ---------------------------------------------------
            boolean fireNow = gamepad2.right_bumper;
            boolean fireEdge = fireNow && !prevFire;
            prevFire = fireNow;

            if (fireEdge) {
                if (shooterLatchedOn && readyStable) {
                    trigger.fire(this);
                } else {
                    // Not ready -> rumble warning
                    gamepad2.rumble(250);
                }
            }

            // --------------------------
            // Minimal telemetry
            // --------------------------
            telemetry.addData("ShooterLatched", shooterLatchedOn ? "ON" : "off");
            telemetry.addData("ToF Range (in)", tofValid ? String.format("%.1f", tofRange) : "—");
            telemetry.addData("RPM ideal", "%.1f", rpmIdeal);
            telemetry.addData("RPM avg", "%.1f", shooter.getAverageRpm());
            telemetry.addData("READY", readyStable ? "YES" : "no");
            telemetry.update();
        }

        // Safety stop
        shooter.stop();
        robot.allStop();
    }
}