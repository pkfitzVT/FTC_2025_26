package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// NEW: import the vision helper
import org.firstinspires.ftc.teamcode.vision.DecodeObeliskVision;
import org.firstinspires.ftc.teamcode.vision.DecodeObeliskVision.Motif;

@TeleOp(name = "TeleopV2_2025", group = "Linear Opmode")
public class TeleopV2_2025 extends LinearOpMode {

    private Bumble robot = new Bumble(telemetry);
    private final Shooter shooter = new Shooter();
    private final distancestuff distanceReader = new distancestuff();

    // Separate trigger class
    private final Trigger trigger = new Trigger();

    // NEW: Decode OBELISK / pattern vision helper
    private DecodeObeliskVision decodeVision;

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

        // NEW: init the vision subsystem (webcam AprilTag → motif)
        decodeVision = new DecodeObeliskVision(telemetry);
        decodeVision.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.addLine("Vision: Point webcam at OBELISK (IDs 21/22/23).");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        double drivePower = -0.3;


        telemetry.addLine("Turning +90 (clockwise)...");
        telemetry.update();
        robot.turnDegrees(90);
        sleep(1000);

        // 4. Test a counterclockwise 90 degree turn
        telemetry.addLine("Turning -90 (counterclockwise)...");
        telemetry.update();
        robot.turnDegrees(-90);
        sleep(1000);

        telemetry.addLine("Turn test complete.");
        telemetry.update();


        // robot.strafeLeftAuto(10);

        //robot.strafeRightAuto(10);



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

            // ===== Trigger (gamepad2.B) =====
            // Simple: hold or tap B to fire; each call blocks, moves out and back.
            if (gamepad2.b) {
                trigger.fire(this);
            }

            // ===== VISION UPDATE: read OBELISK / pattern =====
            decodeVision.update();                       // read current detections
            Motif motif = decodeVision.getCurrentMotif();
            String motifStr = decodeVision.getMotifString();
            char[] rampPattern = decodeVision.getRampPattern(); // 9 indices

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

            // === Pattern / motif telemetry for drive team ===
            telemetry.addLine("\n=== OBELISK / PATTERN INFO ===");
            telemetry.addData("Tag Visible", decodeVision.isTagVisible());
            telemetry.addData("Tag ID", decodeVision.getCurrentTagId());
            telemetry.addData("Motif", motifStr); // "GPP", "PGP", "PPG", or "UNKNOWN");

            telemetry.addData("Ramp Pattern",
                    "%c %c %c   %c %c %c   %c %c %c",
                    rampPattern[0], rampPattern[1], rampPattern[2],
                    rampPattern[3], rampPattern[4], rampPattern[5],
                    rampPattern[6], rampPattern[7], rampPattern[8]);

            telemetry.update();
        }

        // Safety stop
        shooter.stop();
        robot.allStop();

        // Shut down vision to save resources
        decodeVision.close();
    }
}
