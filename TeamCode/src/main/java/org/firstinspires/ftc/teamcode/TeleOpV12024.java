package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// Import the Bumble class

/*
This class was created to integrate the new hardware class, Bumble,
with the teleOp methods.
*/

@TeleOp(name = "TeleOp_2024_v2", group = "Linear Opmode")
public class TeleOpV12024 extends LinearOpMode {
    // Declare OpMode members

    private Bumble robot = new Bumble();

    @Override
    public void runOpMode() {
        // Initialize the hardware variables
        robot.init(hardwareMap);


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //robot.driveForwardAuto(10);


        double drivePower = -.3;

        robot.resetEncoders();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //Drive Controls
            if (gamepad1.right_trigger>0){
                drivePower = -0.5;
            }
            else {
                drivePower = -.3;
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
                if(drivePower<0){
                    robot.rotateLeft(drivePower);
                }
                else {
                    robot.rotateRight(drivePower);
                }
            } else if (gamepad1.left_bumper) {
                if(drivePower<0){
                    robot.rotateRight(drivePower);
                }
                else {
                    robot.rotateLeft(drivePower);
                }
                } else {
                robot.allStop();
            }
            robot.displayMotorEncoders(telemetry);

        }
    }
}

