package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// If Shooter is in a subpackage, keep this import. If it's in the same package, remove the import.


@TeleOp(name = "TeleopV12025", group = "Linear Opmode")
public class TeleopV12025 extends LinearOpMode {

    // ---- Existing hardware wrapper ----
    private Bumble robot = new Bumble(telemetry);

    // ---- ADDED: Shooter subsystem ----
    private final Shooter shooter = new Shooter();

    @Override
    public void runOpMode() {
        // Initialize hardware
        robot.init(hardwareMap);

        // ---- ADDED: init shooter (expects "left_shooter" & "right_shooter" in config) ----
        shooter.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        double drivePower = -0.3;

        robot.resetEncoders();
        robot.setRunWithoutEncoders();

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
                if (drivePower < 0) {
                    robot.rotateLeft(drivePower);
                } else {
                    robot.rotateRight(drivePower);
                }
            } else if (gamepad1.left_bumper) {
                if (drivePower < 0) {
                    robot.rotateRight(drivePower);
                } else {
                    robot.rotateLeft(drivePower);
                }
            } else {
                robot.allStop();
            }

            // ===== Shooter Controls (gamepad2) =====
            // RT = full power, LT = mid power, A = stop
            if (gamepad2.right_trigger > 0.1) {
                shooter.shoot(1.0  );
            } else if (gamepad2.left_trigger > 0.1) {
                shooter.shoot(0.6);
            } else if (gamepad2.a) {
                shooter.stop();
            }
            // (Optional) Fail-safe stop if no trigger pressed:
            // else { shooter.stop(); }

            // Telemetry
            robot.displayMotorEncoders(telemetry);
            String shooterState =
                    (gamepad2.right_trigger > 0.1) ? "FULL" :
                            (gamepad2.left_trigger > 0.1)  ? "MID"  :
                                    (gamepad2.a)                   ? "STOP" : "IDLE";
            telemetry.addData("Shooter", shooterState);
            telemetry.update();
        }

        // Safety stops on exit
        shooter.stop();
        robot.allStop();
    }
}
