package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.DecodeObeliskVision;
import org.firstinspires.ftc.teamcode.DecodeObeliskVision.Motif;

// Dashboard streaming
import com.acmerobotics.dashboard.FtcDashboard;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

@TeleOp(name = "CompSolo2026", group = "Linear Opmode")
public class TeleopV4Solo_2026 extends LinearOpMode {

    private Bumble robot;
    private final ShooterV1 shooter = new ShooterV1();
    private final distancestuff distanceReader = new distancestuff();

    // Separate trigger class
    private final Trigger trigger = new Trigger();

    // Decode OBELISK / pattern vision helper
    private DecodeObeliskVision decodeVision;

    // FTC Dashboard instance
    private FtcDashboard dashboard;


    private final GoalTagTracker blueGoalTracker = new GoalTagTracker(20);


    @Override
    public void runOpMode() {

        dashboard = FtcDashboard.getInstance();

        // Wrap telemetry FIRST
        telemetry = new MultipleTelemetry(super.telemetry, dashboard.getTelemetry());

        // Now build subsystems that store telemetry
        robot = new Bumble(telemetry);

        decodeVision = new DecodeObeliskVision(telemetry);
        decodeVision.init(hardwareMap);

        dashboard.startCameraStream(decodeVision.getVisionPortal(), 30);

        // Init robot + drive
        robot.init(hardwareMap);
        robot.resetEncoders();
        robot.setRunWithoutEncoders();

        shooter.init(hardwareMap);
        trigger.init(hardwareMap);
        distanceReader.init(hardwareMap, "distance_sensor");


        waitForStart();
        if (isStopRequested()) return;


        double drivePower = -0.3;

        while (opModeIsActive()) {
            // ===== Drive Controls (gamepad1) =====
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

            // ===== Shooter flywheels (gamepad1) =====
            if (gamepad1.y) {
                //shooter.shoot(0.35);
                shooter.setTargetRpm(85); // “high”
                //shooter.setTargetRpm(150); // “high”
            } else if (gamepad1.x) {
                //shooter.shoot(0.40);
                shooter.setTargetRpm(75); // “high”
                //shooter.setTargetRpm(200); // “high”

            } else if (gamepad1.a) {
                shooter.stop();
            }


            telemetry.addData("g2 RT/LT", "%.2f / %.2f",
                    gamepad1.right_trigger, gamepad1.left_trigger);

            // ===== Trigger (gamepad2.B) =====
            if (gamepad1.b) {
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

            // ===== Vision: motif + distance/heading =====
            if (decodeVision != null) {
                // Update vision state
                decodeVision.update();

                blueGoalTracker.update(decodeVision.getDetections());

                Motif motif = decodeVision.getCurrentMotif();
                String motifStr = decodeVision.getMotifString();
                char[] rampPattern = decodeVision.getRampPattern();

                telemetry.addLine("\n=== OBELISK / PATTERN INFO ===");
                telemetry.addData("Tag Visible", decodeVision.isTagVisible());
                telemetry.addData("Tag ID", decodeVision.getCurrentTagId());
                telemetry.addData("Motif", motifStr);
                telemetry.addData("Ramp Pattern",
                        "%c %c %c   %c %c %c   %c %c %c",
                        rampPattern[0], rampPattern[1], rampPattern[2],
                        rampPattern[3], rampPattern[4], rampPattern[5],
                        rampPattern[6], rampPattern[7], rampPattern[8]);

                telemetry.addLine("\n=== BLUE GOAL TAG ===");
                telemetry.addData("Goal Visible", blueGoalTracker.isGoalVisible());
                telemetry.addData("Goal ID", blueGoalTracker.getGoalId());

                Double gr = blueGoalTracker.getRangeInches();
                Double gb = blueGoalTracker.getBearingDeg();
                if (gr != null && gb != null) {
                    telemetry.addData("Goal Range (in)", "%.1f", gr);
                    telemetry.addData("Goal Bearing (deg)", "%.1f", gb);
                } else {
                    telemetry.addData("Goal Range/Bearing", "No tag pose");
                }

                boolean inRange = blueGoalTracker.isInRange(30.0, -10.0, 10.0);
                telemetry.addData("IN RANGE?", inRange);



                // NEW: distance and heading from AprilTag
                Double range = decodeVision.getCurrentRangeInches();
                Double bearing = decodeVision.getCurrentBearingDeg();
                if (range != null && bearing != null) {
                    telemetry.addData("Tag Range (in)", "%.1f", range);
                    telemetry.addData("Tag Bearing (deg)", "%.1f", bearing);
                } else {
                    telemetry.addData("Tag Range/ Bearing", "No tag pose");
                }
            }

            telemetry.update();
        }

        // Safety stop
        shooter.stop();
        robot.allStop();

        // Shut down vision & dashboard to save resources
        if (decodeVision != null) {
            decodeVision.close();
        }
        if (dashboard != null) {
            dashboard.stopCameraStream();
        }
    }
}
