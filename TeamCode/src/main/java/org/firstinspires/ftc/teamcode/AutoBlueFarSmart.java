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
@Autonomous(name = "AutoBlueFarSmart", group = "Decode")
public class AutoBlueFarSmart extends LinearOpMode {

    // --- Your robot helper classes ---
    private Bumble bumble;
    private ShooterV1 shooter;
    private Trigger trigger;

    //Auto
    private final distancestuff distanceReader = new distancestuff();

    // --- Shooting parameters ---
    private static final double TARGET_RPM = 100.0;

    // How close we need to be to call it "at speed"
    private static final double RPM_TOLERANCE = 2.15;

    // Require "at speed" continuously for this long before we shoot
    private static final long STABLE_MS = 250;

    // Safety timeout so we don't wait forever if something is wrong
    private static final double SPINUP_TIMEOUT_SEC = 7.0;

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

        distanceReader.init(hardwareMap, "distance_sensor");
        bumble.setCollisionSafety(10.0, 1500, 40, 2);


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


        telemetry.addLine("Step 1: Drive forward 72 inches");
        telemetry.addData("Safety maxWaitMs", bumble.getMaxWaitMs());
        telemetry.update();
        sleep(250);

        //drive forward safe
        boolean ok = bumble.driveForwardAutoSafe(this, distanceReader, 72, 0.3);

        telemetry.addLine("Step 2: Rotate 45 degrees");
        telemetry.update();
        bumble.turnDegrees(47);
        bumble.allStop();

        telemetry.addData("Step 3", "Spin flywheel to %.1f RPM", TARGET_RPM);
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

        telemetry.addLine("Step 4: Shooter ready. Firing 3x with re-check...");
        telemetry.update();

        for (int i = 1; i <= 3 && opModeIsActive(); i++) {

            // Reset per-shot timers/flags so every shot must re-stabilize
            ElapsedTime overallTimerShot = new ElapsedTime();
            ElapsedTime stableTimerShot  = new ElapsedTime();
            boolean stableWindowStartedShot = false;

            while (opModeIsActive() && overallTimerShot.seconds() < SPINUP_TIMEOUT_SEC) {

                boolean atSpeed = shooter.isAtSpeed(RPM_TOLERANCE);

                if (atSpeed) {
                    if (!stableWindowStartedShot) {
                        stableWindowStartedShot = true;
                        stableTimerShot.reset();
                    }
                } else {
                    stableWindowStartedShot = false;
                }

                telemetry.addData("Shot", "%d/3", i);
                telemetry.addData("Target RPM", TARGET_RPM);
                telemetry.addData("Left RPM", "%.1f", shooter.getLeftRpm());
                telemetry.addData("Right RPM", "%.1f", shooter.getRightRpm());
                telemetry.addData("Avg RPM", "%.1f", shooter.getAverageRpm());
                telemetry.addData("Stable time (ms)", stableWindowStartedShot ? (long) stableTimerShot.milliseconds() : 0);
                telemetry.addData("Spinup timeout (s)", "%.1f / %.1f", overallTimerShot.seconds(), SPINUP_TIMEOUT_SEC);
                telemetry.update();

                if (stableWindowStartedShot && stableTimerShot.milliseconds() >= STABLE_MS) {
                    break;
                }

                sleep(20);
            }

            // If we didn't stabilize in time, abort remaining shots
            if (!shooter.isAtSpeed(RPM_TOLERANCE)) {
                telemetry.addLine("Shooter not stable. Aborting remaining shots.");
                telemetry.update();
                break;
            }

            // Fire this shot
            trigger.fire(this);

            // Let RPM dip and recover (tune this)
            sleep(250);
        }




        // ----------------------------
        // D) SHUTDOWN
        // ----------------------------
        telemetry.addLine("Step 5: Stopping flywheel and ending auto.");
        telemetry.update();
        shooter.stop();
        bumble.allStop();

        telemetry.addLine("Step 6: Strafe Left 15 inches");
        telemetry.update();
        bumble.strafeRightAuto(15);  // uses your encoder logic
        bumble.allStop();

        telemetry.addLine("AUTO COMPLETE.");
        telemetry.update();
        bumble.allStop();

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

