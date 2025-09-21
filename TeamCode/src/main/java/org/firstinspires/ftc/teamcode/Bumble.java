package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
This class was created to help simplify the code for the robot.
All of the hardware mapping is done in this class.
 */


public class Bumble {
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    // Telemetry object
    private Telemetry telemetry;

    // Constructor to initialize telemetry
    public Bumble(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    HardwareMap hwMap = null;
   // Constructor
    public Bumble() {
    }
    // Initialize standard Hardware interfaces
    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        leftFrontDrive = hwMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hwMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hwMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hwMap.get(DcMotor.class, "right_back_drive");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
  }
    public void init(HardwareMap ahwMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        init(ahwMap);
    }
    public void setRunWithoutEncoders() {
        leftFrontDrive .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive  .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void driveForward (double drivePower) {
        leftFrontDrive.setPower(drivePower);
        rightFrontDrive.setPower(drivePower);
        leftBackDrive.setPower(drivePower);
        rightBackDrive.setPower(drivePower);
    }

    public void driveBackward (double drivePower) {
        leftFrontDrive.setPower(-drivePower);
        rightFrontDrive.setPower(-drivePower);
        leftBackDrive.setPower(-drivePower);
        rightBackDrive.setPower(-drivePower);
    }


    public void strafeRight (double drivePower) {
        leftFrontDrive.setPower(drivePower);
        rightFrontDrive.setPower(-drivePower);
        leftBackDrive.setPower(-drivePower);
        rightBackDrive.setPower(drivePower);
    }

    public void strafeLeft (double drivePower) {
        leftFrontDrive.setPower(-drivePower);
        rightFrontDrive.setPower(drivePower);
        leftBackDrive.setPower(drivePower);
        rightBackDrive.setPower(-drivePower);
    }

    public void rotateRight (double drivePower) {
        leftFrontDrive.setPower(drivePower);
        rightFrontDrive.setPower(-drivePower);
        leftBackDrive.setPower(drivePower);
        rightBackDrive.setPower(-drivePower);
    }

    public void rotateLeft (double drivePower) {
        leftFrontDrive.setPower(-drivePower);
        rightFrontDrive.setPower(drivePower);
        leftBackDrive.setPower(-drivePower);
        rightBackDrive.setPower(drivePower);
    }
    public void allStop() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }


    // Method to display motor encoder values
    public void displayMotorEncoders(Telemetry telemetry) {
        telemetry.addData("Left Front Encoder", leftFrontDrive.getCurrentPosition());
        telemetry.addData("Left Back Encoder", leftBackDrive.getCurrentPosition());
        telemetry.addData("Right Front Encoder", rightFrontDrive.getCurrentPosition());
        telemetry.addData("Right Back Encoder", rightBackDrive.getCurrentPosition());
        telemetry.update();
    }


    public void resetEncoders() {
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // After resetting, set the motors to RUN_WITHOUT_ENCODER mode
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void setRunEncoders(){
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    //autonomous methods
    public void driveForwardAuto(int target) {
        resetEncoders();

        // Calculate the target encoder clicks
        int targetClicks = (int) (target * 42.1);

        // Initialize power
        double basePower = 0.2;

        while (absAverageMotorEncoders() < targetClicks) {
            // Calculate the difference in encoder values
            int leftSidePosition = (leftFrontDrive.getCurrentPosition() + leftBackDrive.getCurrentPosition()) / 2;
            int rightSidePosition = (rightFrontDrive.getCurrentPosition() + rightBackDrive.getCurrentPosition()) / 2;
            int error = leftSidePosition - rightSidePosition;

            // Correction factor
            double correction = error * 0.001; // Tuning parameter (adjust for sensitivity)

            // Adjust motor powers
            double leftPower = basePower - correction;
            double rightPower = basePower + correction;

            // Apply adjusted powers
            leftFrontDrive.setPower(leftPower);
            leftBackDrive.setPower(leftPower);
            rightFrontDrive.setPower(rightPower);
            rightBackDrive.setPower(rightPower);

            //
            // Add telemetry updates
            telemetry.addData("Target Clicks", targetClicks);
            telemetry.addData("Average Encoder", averageMotorEncoders());
            telemetry.addData("Left Side Encoder", leftSidePosition);
            telemetry.addData("Right Side Encoder", rightSidePosition);
            telemetry.addData("Error", error);
            telemetry.update();
        }
        allStop();
    }
    public void driveBackwardAuto(int target) {
        // Calculate the target encoder clicks
        resetEncoders();
        int targetClicks = (int) (target * 42.1);

        while (Math.abs(averageMotorEncoders()) < targetClicks) {
            // Calculate encoder positions for left and right sides
            int leftSidePosition = (leftFrontDrive.getCurrentPosition() + leftBackDrive.getCurrentPosition()) / 2;
            int rightSidePosition = (rightFrontDrive.getCurrentPosition() + rightBackDrive.getCurrentPosition()) / 2;
            int error = leftSidePosition - rightSidePosition;

            // Correction factor (tuning parameter may require adjustment)
            double correction = error * 0.001;

            // Adjust motor powers for backward motion
            double leftPower = -0.2 - correction; // Negative power for backward
            double rightPower = -0.2 + correction;

            // Apply adjusted powers
            leftFrontDrive.setPower(leftPower);
            leftBackDrive.setPower(leftPower);
            rightFrontDrive.setPower(rightPower);
            rightBackDrive.setPower(rightPower);

            // Add telemetry for monitoring
            if (telemetry != null) {
                telemetry.addData("Target Clicks", targetClicks);
                telemetry.addData("Average Encoder", Math.abs(averageMotorEncoders()));
                telemetry.addData("Left Encoder", leftSidePosition);
                telemetry.addData("Right Encoder", rightSidePosition);
                telemetry.addData("Error", error);
                telemetry.update();
            }
        }

        // Stop the robot
       allStop();
    }
    public void strafeRightAuto(int target) {

        //reset encoders
        resetEncoders();
        // Calculate the target encoder clicks
        //int targetClicks = (int) (target * 42.1);
        int targetClicks = target;

        while (Math.abs(averageMotorEncoders()) < targetClicks) {
            // Calculate encoder positions for left and right sides
            int leftSidePosition = (leftFrontDrive.getCurrentPosition() + leftBackDrive.getCurrentPosition()) / 2;
            int rightSidePosition = (rightFrontDrive.getCurrentPosition() + rightBackDrive.getCurrentPosition()) / 2;
            int error = leftSidePosition - rightSidePosition;

            // Correction factor (adjust as needed for strafing)
            double correction = error * 0.001;

            // Adjust motor powers for strafing right
            // Mecanum wheel pattern for strafing
            double leftFrontPower = 0.2 - correction;
            double leftBackPower = -0.2 - correction;
            double rightFrontPower = -0.2 + correction;
            double rightBackPower = 0.2 + correction;

            // Apply motor powers
            leftFrontDrive.setPower(leftFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightFrontDrive.setPower(rightFrontPower);
            rightBackDrive.setPower(rightBackPower);

            // Add telemetry for monitoring
            if (telemetry != null) {
                telemetry.addData("Target Clicks", targetClicks);
                telemetry.addData("Average Encoder", Math.abs(averageMotorEncoders()));
                telemetry.addData("Left Encoder", leftSidePosition);
                telemetry.addData("Right Encoder", rightSidePosition);
                telemetry.addData("Error", error);
                telemetry.update();
            }
        }

        // Stop the robot
        allStop();
    }

    public void strafeLeftAuto(int target) {
        //reset encoders
        resetEncoders();

        // Calculate the target encoder clicks
        int targetClicks = (int) (target*(300/6.5));

        // Reset encoders to start from zero
        resetEncoders();

        while (absAverageMotorEncoders() < targetClicks) {
            // Calculate encoder positions for left and right sides
            int leftSidePosition = (leftFrontDrive.getCurrentPosition() + leftBackDrive.getCurrentPosition()) / 2;
            int rightSidePosition = (rightFrontDrive.getCurrentPosition() + rightBackDrive.getCurrentPosition()) / 2;
            int error = leftSidePosition - rightSidePosition;

            // Correction factor (adjust as needed for strafing)
            double correction = error * 0.001;

            // Mecanum wheel power pattern for strafing left
            double leftFrontPower = -0.2 - correction;
            double leftBackPower = 0.2 - correction;
            double rightFrontPower = 0.2 + correction;
            double rightBackPower = -0.2 + correction;

            // Apply motor powers
            leftFrontDrive.setPower(leftFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightFrontDrive.setPower(rightFrontPower);
            rightBackDrive.setPower(rightBackPower);

            // Add telemetry for debugging
            if (telemetry != null) {
                telemetry.addData("Target Clicks", targetClicks);
                telemetry.addData("Average Encoder", absAverageMotorEncoders());
                telemetry.addData("Left Encoder", leftSidePosition);
                telemetry.addData("Right Encoder", rightSidePosition);
                telemetry.addData("Error", error);
                telemetry.update();
            }
        }

        // Stop the robot
        allStop();
    }

    /*
    public void driveForwardAuto(int target){
        resetEncoders();
        // Calculate the target encoder clicks
        int targetClicks = (int) (target * 42.1);
        while(averageMotorEncoders()<targetClicks){
            driveForward(.2);
        }

    }
    public void driveBackwardAuto(int target) {

       resetEncoders();
        // Calculate the target encoder clicks
        int targetClicks = (int) (target * 42.1);

        // While loop to move backward until the target is reached
        while (Math.abs(averageMotorEncoders()) < targetClicks) {
            driveBackward(0.2); // Negative power for backward motion
        }
    }
    */


    public int averageMotorEncoders() {
        int leftFrontPosition = leftFrontDrive.getCurrentPosition();
        int leftBackPosition = leftBackDrive.getCurrentPosition();
        int rightFrontPosition = rightFrontDrive.getCurrentPosition();
        int rightBackPosition = rightBackDrive.getCurrentPosition();
        int averagePosition = (leftFrontPosition + leftBackPosition + rightFrontPosition + rightBackPosition) / 4;
        return averagePosition;
    }

    public int absAverageMotorEncoders() {
        int leftFrontPosition = Math.abs(leftFrontDrive.getCurrentPosition());
        int leftBackPosition = Math.abs(leftBackDrive.getCurrentPosition());
        int rightFrontPosition = Math.abs(rightFrontDrive.getCurrentPosition());
        int rightBackPosition = Math.abs(rightBackDrive.getCurrentPosition());
        int averagePosition = (leftFrontPosition + leftBackPosition + rightFrontPosition + rightBackPosition) / 4;
        return averagePosition;
    }

}
