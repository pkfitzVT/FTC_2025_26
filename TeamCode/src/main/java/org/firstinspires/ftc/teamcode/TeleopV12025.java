package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleopV12025", group = "Linear Opmode")
public class TeleopV12025 extends LinearOpMode {

    // ---- Existing hardware wrapper ----
    private Bumble robot = new Bumble(telemetry);

    // ---- Shooter subsystem ----
    private final Shooter shooter = new Shooter();

    // ---- NEW: distance sensor wrapper (your class) ----
    private final distancestuff distanceReader = new distancestuff();

    @Override
    public void runOpMode() {
        // Initialize hardware
        robot.init(hardwareMap);
        shooter.init(hardwareMap);  // expects: left_shooter, right_shooter, trigger

        // Init distance sensor; RC config name must be "distance_sensor"
        distanceReader.init(hardwareMap, "distance_sensor");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        double drivePower = -0.3;

        robot.resetEncoders();
        robot.setRunWithoutEncoders();

        // --- Edge-detect for B button so one press = one trigger cycle ---
        boolean prevB = false;

        while (opModeIsActive()) {
            // ===== Drive Controls (gamepad1) =====
            if (gamepad1.left_trigger > 0) {
                drivePower = -0.2;
            } else if (gamepad1.right_trigger > 0) {
                drivePower = -0.5;
            } else {
                drivePower = -0.3;
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

            // ===== Shooter flywheels (gamepad2) =====
            if (gamepad2.right_trigger > 0.1) {
                shooter.shoot(0.35);
            } else if (gamepad2.left_trigger > 0.1) {
                shooter.shoot(0.40);
            } else if (gamepad2.a) {
                shooter.stop();
            }

            // ===== Trigger servo (gamepad2.B) → 90° poke then back =====
            boolean b = gamepad2.b;
            if (b && !prevB) {
                shooter.fireTrigger(this); // rotates ~90°, dwells, returns
            }
            prevB = b;

            // ===== Telemetry =====
            robot.displayMotorEncoders(telemetry);
            String shooterState =
                    (gamepad2.right_trigger > 0.1) ? "PWR 0.35" :
                            (gamepad2.left_trigger  > 0.1) ? "PWR 0.40" :
                                    (gamepad2.a)                   ? "STOP"     : "IDLE";
            telemetry.addData("Shooter", shooterState);
            telemetry.addData("TriggerPos", "%.2f", shooter.getTriggerPosition());

            // NEW: Distance in inches from your distancestuff class
            double inches = distanceReader.getDistanceInches();
            if (Double.isNaN(inches)) {
                telemetry.addData("Distance (in)", "—");
            } else {
                telemetry.addData("Distance (in)", "%.1f", inches);
            }

            telemetry.update();
        }

        // Safety stops on exit
        shooter.stop();
        shooter.triggerHome();
        robot.allStop();
    }
}
