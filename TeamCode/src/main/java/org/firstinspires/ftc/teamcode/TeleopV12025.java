package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleopV12025", group = "Linear Opmode")
public class TeleopV12025 extends LinearOpMode {

    private Bumble robot = new Bumble(telemetry);
    private final Shooter shooter = new Shooter();
    private final distancestuff distanceReader = new distancestuff();

    // NEW: separate trigger class
    private final Trigger trigger = new Trigger();

    @Override
    public void runOpMode() {
        // Init robot + drive
        robot.init(hardwareMap);
        robot.resetEncoders();
        robot.setRunWithoutEncoders();

        // Init shooter wheels
        shooter.init(hardwareMap);

        // Init trigger servo
        trigger.init(hardwareMap);

        // Init distance sensor
        distanceReader.init(hardwareMap, "distance_sensor");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        double drivePower = -0.3;

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
                shooter.shoot(0.30);
            } else if (gamepad2.left_trigger > 0.1) {
                shooter.shoot(0.32  );
            } else if (gamepad2.a) {
                shooter.stop();
            }

            // ===== Trigger (gamepad2.B) =====
            // Simple: hold or tap B to fire; each call blocks, moves out and back.
            if (gamepad2.b) {
                trigger.fire(this);
            }

            // ===== Telemetry =====
            double leftRpm  = shooter.getLeftRpm();
            double rightRpm = shooter.getRightRpm();
            double avgRpm   = shooter.getAverageRpm();

            robot.displayMotorEncoders(telemetry);

            telemetry.addData("Shooter RPM L/R", "%.1f / %.1f", leftRpm, rightRpm);
            telemetry.addData("Shooter RPM Avg", "%.1f", avgRpm);

            telemetry.addData("TriggerPos", "%.2f", trigger.getPosition());
            telemetry.addData("gamepad2.b", gamepad2.b);

            double inches = distanceReader.getDistanceInches();
            if (Double.isNaN(inches)) {
                telemetry.addData("Distance (in)", "—");
            } else {
                telemetry.addData("Distance (in)", "%.1f", inches);
            }

            telemetry.update();
        }

        // Safety stop
        shooter.stop();
        robot.allStop();
    }
}
