package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * IMPORTANT NOTES FOR NEW TEAMS:
 * - Bumble's driveForwardAuto() and turnDegrees() are "blocking" loops.
 *   That means once we call them, the OpMode can't do anything else until they finish.
 *   We'll keep our Autonomous simple and reliable using your existing methods.
 * - Shooter spin-up verification IS stop-safe because we check opModeIsActive() in that loop.
 */
@Autonomous(name = "AutoRed03", group = "Decode")
public class AutoRed03 extends LinearOpMode {

    // --- Your robot helper classes ---
    private Bumble bumble;
    private ShooterV1 shooter;
    private Trigger trigger;

    // --- Shooting parameters ---
    private static final double TARGET_RPM = 80.0;

    // How close we need to be to call it "at speed"
    private static final double RPM_TOLERANCE = 5.0;

    // Require "at speed" continuously for this long before we shoot
    private static final long STABLE_MS = 250;

    // Safety timeout so we don't wait forever if something is wrong
    private static final double SPINUP_TIMEOUT_SEC = 5.0;

    @Override
    public void runOpMode() {

        // 1) Create objects
        bumble = new Bumble();
        shooter = new ShooterV1();
        trigger = new Trigger();

        // 2) Initialize hardware through your classes
        // Bumble has an overload that accepts telemetry, so it can print encoder + heading info.
        bumble.init(hardwareMap, telemetry);
        shooter.init(hardwareMap);
        trigger.init(hardwareMap);

        // 3) Pre-start telemetry (shows up while you are on the INIT screen)
        telemetry.addLine("AutoDriveTurnShoot READY");
        telemetry.addLine("Plan: Forward 48 -> CW 45 -> Forward 55 -> Spin 100 RPM -> Fire");
        telemetry.update();

        // Wait for the driver to press PLAY
        waitForStart();

        // If STOP was pressed immediately, exit cleanly
        if (!opModeIsActive()) return;

        // ----------------------------
        // A) DRIVE PATH
        // ----------------------------

        telemetry.addLine("Step 1: Drive forward 10 inches");
        telemetry.update();
        bumble.driveForwardAuto(10);  // uses your encoder logic
        bumble.allStop();

        telemetry.addLine("Step 2: Strafe left ");
        telemetry.update();
        bumble.strafeLeftAuto(48);  // uses your encoder logic
        bumble.allStop();

        telemetry.addLine("Step 3: Drive forward ");
        telemetry.update();
        bumble.driveForwardAuto(24);  // uses your encoder logic
        bumble.allStop();

        telemetry.addLine("Step 4: Strafe left ");
        telemetry.update();
        bumble.strafeLeftAuto(16);  // uses your encoder logic
        bumble.allStop();

        telemetry.addLine("Step 4.5: Turn ");
        telemetry.update();
        bumble.turnDegrees(5);  // uses your encoder logic
        bumble.allStop();

        telemetry.addLine("Step 5: Drive forward ");
        telemetry.update();
        bumble.driveForwardAuto(35);  // uses your encoder logic
        bumble.allStop();

        telemetry.addLine("Step 5.5: Turn ");
        telemetry.update();
        bumble.turnDegrees(-5 );  // uses your encoder logic
        bumble.allStop();


        telemetry.addData("Step 6", "Spin flywheel to %.1f RPM", TARGET_RPM);
        telemetry.update();

        shooter.setTargetRpm(TARGET_RPM);

        // We will require the shooter to be "at speed" for STABLE_MS continuously.
        ElapsedTime overallTimer = new ElapsedTime();
        ElapsedTime stableTimer = new ElapsedTime();

        boolean stableWindowStarted = false;

        while (opModeIsActive() && overallTimer.seconds() < SPINUP_TIMEOUT_SEC) {

            // Read RPM values (these come from ShooterV1's getVelocity() conversions)
            double leftRpm = shooter.getLeftRpm();
            double rightRpm = shooter.getRightRpm();
            double avgRpm = shooter.getAverageRpm();

            boolean atSpeed = shooter.isAtSpeed(RPM_TOLERANCE);

            // If we just became atSpeed, start the stable timer
            if (atSpeed) {
                if (!stableWindowStarted) {
                    stableWindowStarted = true;
                    stableTimer.reset();
                }
            } else {
                // Not at speed -> reset stability window
                stableWindowStarted = false;
            }

            // Telemetry so you can see what the flywheel is doing
            telemetry.addData("Target RPM", TARGET_RPM);
            telemetry.addData("Left RPM", "%.1f", leftRpm);
            telemetry.addData("Right RPM", "%.1f", rightRpm);
            telemetry.addData("Avg RPM", "%.1f", avgRpm);
            //telemetry.addData("At speed? (±%.1f)", RPM_TOLERANCE, atSpeed);
            telemetry.addData("Stable time (ms)", stableWindowStarted ? (long) (stableTimer.milliseconds()) : 0);
            telemetry.addData("Spinup timeout (s)", "%.1f / %.1f", overallTimer.seconds(), SPINUP_TIMEOUT_SEC);
            telemetry.update();

            // If we have been at speed long enough, we are ready to shoot
            if (stableWindowStarted && stableTimer.milliseconds() >= STABLE_MS) {
                break;
            }

            // Small pause keeps telemetry readable and reduces CPU load
            sleep(20);
        }

        // If we timed out, we can choose to NOT shoot.
        // New-team-friendly behavior: if not stable, abort the shot.
        boolean readyToShoot = shooter.isAtSpeed(RPM_TOLERANCE);
        if (!readyToShoot) {
            telemetry.addLine("Shooter NOT stable at speed. Aborting shot for safety.");
            telemetry.addLine("Check: motor direction, TICKS_PER_REV, battery, friction, wiring.");
            telemetry.update();

            shooter.stop();
            bumble.allStop();
            return;

        }

        if (!opModeIsActive()) return;

        // ----------------------------
        // C) FIRE ONCE
        // ----------------------------
        telemetry.addLine("Step 7: Shooter ready. Firing trigger once...");
        telemetry.update();

        // Trigger.fire() uses opMode.sleep() internally, so we pass "this"
        trigger.fire(this);

        // ----------------------------
        // D) SHUTDOWN
        // ----------------------------
        telemetry.addLine("Step 8: Stopping flywheel and ending auto.");
        telemetry.update();

        shooter.stop();
        bumble.allStop();

        telemetry.addLine("AUTO COMPLETE.");
        telemetry.update();

        // Optional: pause briefly so you can read final telemetry
        sleep(500);

        /*
        if (!opModeIsActive()) return;

        telemetry.addLine("Step 2: Turn clockwise 45 degrees");
        telemetry.update();
        bumble.turnDegrees(45);       // your Bumble: positive degrees = clockwise
        bumble.allStop();

        if (!opModeIsActive()) return;

        telemetry.addLine("Step 3: Drive forward 55 inches");
        telemetry.update();
        bumble.driveForwardAuto(10);
        bumble.allStop();

        if (!opModeIsActive()) return;

        telemetry.addLine("Drive path complete. Robot stopped.");
        telemetry.update();

        // ----------------------------
        // B) SPIN UP FLYWHEEL + VERIFY
        // ----------------------------

        telemetry.addData("Step 4", "Spin flywheel to %.1f RPM", TARGET_RPM);
        telemetry.update();

        shooter.setTargetRpm(TARGET_RPM);

        // We will require the shooter to be "at speed" for STABLE_MS continuously.
        ElapsedTime overallTimer = new ElapsedTime();
        ElapsedTime stableTimer  = new ElapsedTime();

        boolean stableWindowStarted = false;

        while (opModeIsActive() && overallTimer.seconds() < SPINUP_TIMEOUT_SEC) {

            // Read RPM values (these come from ShooterV1's getVelocity() conversions)
            double leftRpm  = shooter.getLeftRpm();
            double rightRpm = shooter.getRightRpm();
            double avgRpm   = shooter.getAverageRpm();

            boolean atSpeed = shooter.isAtSpeed(RPM_TOLERANCE);

            // If we just became atSpeed, start the stable timer
            if (atSpeed) {
                if (!stableWindowStarted) {
                    stableWindowStarted = true;
                    stableTimer.reset();
                }
            } else {
                // Not at speed -> reset stability window
                stableWindowStarted = false;
            }

            // Telemetry so you can see what the flywheel is doing
            telemetry.addData("Target RPM", TARGET_RPM);
            telemetry.addData("Left RPM", "%.1f", leftRpm);
            telemetry.addData("Right RPM", "%.1f", rightRpm);
            telemetry.addData("Avg RPM", "%.1f", avgRpm);
            //telemetry.addData("At speed? (±%.1f)", RPM_TOLERANCE, atSpeed);
            telemetry.addData("Stable time (ms)", stableWindowStarted ? (long)(stableTimer.milliseconds()) : 0);
            telemetry.addData("Spinup timeout (s)", "%.1f / %.1f", overallTimer.seconds(), SPINUP_TIMEOUT_SEC);
            telemetry.update();

            // If we have been at speed long enough, we are ready to shoot
            if (stableWindowStarted && stableTimer.milliseconds() >= STABLE_MS) {
                break;
            }

            // Small pause keeps telemetry readable and reduces CPU load
            sleep(20);
        }

        // If we timed out, we can choose to NOT shoot.
        // New-team-friendly behavior: if not stable, abort the shot.
        boolean readyToShoot = shooter.isAtSpeed(RPM_TOLERANCE);
        if (!readyToShoot) {
            telemetry.addLine("Shooter NOT stable at speed. Aborting shot for safety.");
            telemetry.addLine("Check: motor direction, TICKS_PER_REV, battery, friction, wiring.");
            telemetry.update();

            shooter.stop();
            bumble.allStop();
            return;
        }

        if (!opModeIsActive()) return;

        // ----------------------------
        // C) FIRE ONCE
        // ----------------------------
        telemetry.addLine("Step 5: Shooter ready. Firing trigger once...");
        telemetry.update();

        // Trigger.fire() uses opMode.sleep() internally, so we pass "this"
        trigger.fire(this);

        // ----------------------------
        // D) SHUTDOWN
        // ----------------------------
        telemetry.addLine("Step 6: Stopping flywheel and ending auto.");
        telemetry.update();

        shooter.stop();
        bumble.allStop();

        telemetry.addLine("AUTO COMPLETE.");
        telemetry.update();

        // Optional: pause briefly so you can read final telemetry
        sleep(500);
        */

    }
}
