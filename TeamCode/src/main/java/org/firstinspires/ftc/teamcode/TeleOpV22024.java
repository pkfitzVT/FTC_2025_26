package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// Import the Bumble class

/*
This class was created to integrate the new hardware class, Bumble,
with the teleOp methods.
*/

//@TeleOp(name = "TeleOp_2024_v3", group = "Linear Opmode")
public class TeleOpV22024 extends LinearOpMode {
    // Declare OpMode members

    private Bumble robot = new Bumble(telemetry);


    @Override
    public void runOpMode() {
        // Initialize the hardware variables
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //Autonomous Commands Go Here

        //robot.driveForwardAuto(20);


        //robot.driveForwardAuto(48);



        double drivePower = -.3;


        robot.resetEncoders();      // clears any old STOP_AND_RESET
        robot.setRunWithoutEncoders();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //Drive Controls
            if (gamepad1.left_trigger>0){
                drivePower = -0.2;
            }
            else if (gamepad1.right_trigger>0){
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

