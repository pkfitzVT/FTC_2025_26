package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;



/*
This class was created to help simplify the code for the robot.
All of the hardware mapping is done in this class.
 */


public class Bumble {
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    // ============================
// Collision safety (tunable)
// ============================
    private double stopDistanceIn = 10.0;
    private long maxWaitMs = 10000;
    private long waitStepMs = 40;
    private int blockedConfirmReads = 2;

    public long getMaxWaitMs() { return maxWaitMs; }


    // Telemetry object
    private Telemetry telemetry;

    private IMU imu;

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

        // -------- IMU SETUP --------
        imu = hwMap.get(IMU.class, "imu");  // Name must match your config

        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        // You said: hub facing LEFT, USB on the outside.
                        // Adjust these if turns look wrong.
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(imuParams);

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

    public void setCollisionSafety(double stopDistanceIn, long maxWaitMs, long waitStepMs, int confirmReads) {
        this.stopDistanceIn = stopDistanceIn;
        this.maxWaitMs = maxWaitMs;
        this.waitStepMs = waitStepMs;
        this.blockedConfirmReads = Math.max(1, confirmReads);
    }

    private boolean isBlockedStable(distancestuff distanceReader) {
        int blockedCount = 0;

        for (int i = 0; i < blockedConfirmReads; i++) {
            double d = distanceReader.getDistanceInches();
            boolean blockedNow = (!Double.isNaN(d) && d > 0 && d < stopDistanceIn);
            if (blockedNow) blockedCount++;
        }
        return blockedCount >= blockedConfirmReads;
    }

    private boolean waitUntilClear(com.qualcomm.robotcore.eventloop.opmode.LinearOpMode opMode,
                                   distancestuff distanceReader,
                                   String label) {
        long start = System.currentTimeMillis();

        while (opMode.opModeIsActive()) {
            boolean blocked = isBlockedStable(distanceReader);

            if (telemetry != null) {
                double d = distanceReader.getDistanceInches();
                telemetry.addData("COLLISION", "%s | d(in)=%.1f | blocked=%s",
                        label,
                        Double.isNaN(d) ? -1.0 : d,
                        blocked);
                telemetry.update();
            }

            if (!blocked) return true;

            allStop();
            opMode.sleep(waitStepMs);
            
        }
        return false;
    }

    //autonomous methods
    //The target distance should be in ins and will be converted to encoder clicks (41.2 clicks=1in)
    public void driveBackwardAuto(int target) {
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

        }
        allStop();
    }
    public void driveForwardAuto(int target) {
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

            }
        }

        // Stop the robot
       allStop();
    }

    public boolean driveForwardAutoSafe(com.qualcomm.robotcore.eventloop.opmode.LinearOpMode opMode,
                                        distancestuff distanceReader,
                                        int targetInches,
                                        double basePower) {

        resetEncoders();
        int targetClicks = (int) (targetInches * 42.1);

        double forwardPower = -Math.abs(basePower);

        while (opMode.opModeIsActive() && Math.abs(averageMotorEncoders()) < targetClicks) {

            if (distanceReader != null && isBlockedStable(distanceReader)) {

                allStop();

                boolean cleared = waitUntilClear(opMode, distanceReader, "driveForward");
                if (!cleared) {
                    allStop();
                    return false;
                }
            }

            int leftSidePosition  = (leftFrontDrive.getCurrentPosition() + leftBackDrive.getCurrentPosition()) / 2;
            int rightSidePosition = (rightFrontDrive.getCurrentPosition() + rightBackDrive.getCurrentPosition()) / 2;
            int error = leftSidePosition - rightSidePosition;

            double correction = error * 0.001;

            double leftPower  = forwardPower - correction;
            double rightPower = forwardPower + correction;

            leftFrontDrive.setPower(leftPower);
            leftBackDrive.setPower(leftPower);
            rightFrontDrive.setPower(rightPower);
            rightBackDrive.setPower(rightPower);

            if (telemetry != null) {
                telemetry.addData("SAFE FWD TargetClicks", targetClicks);
                telemetry.addData("SAFE FWD AvgEnc", Math.abs(averageMotorEncoders()));
                telemetry.addData("SAFE FWD Error", error);
                telemetry.update();
            }

            opMode.sleep(10);
        }

        allStop();
        return opMode.opModeIsActive();
    }

    public void strafeLeftAuto(int target) {

        //reset encoders
        resetEncoders();
        // Calculate the target encoder clicks
        int targetClicks = (int) (target*(300/6.5));


        while (absAverageMotorEncoders() < targetClicks) {
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

            }
        }

        // Stop the robot
        allStop();
    }

    public void strafeRightAuto(int target) {
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

//imu methods
    public double getHeadingDegrees() {
        // Yaw = rotation around vertical axis (heading)
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    /**
     * Turn the robot by a specified number of degrees using the IMU.
     * Positive degrees = clockwise
     * Negative degrees = counterclockwise
     */
    public void turnDegrees(double degrees) {
        // Reset yaw so we start from 0 for this turn
        imu.resetYaw();

        // FTC convention: IMU yaw is positive CCW.
        // We want: positive input = CW.
        // So the yaw target is the negative of the requested degrees.
        double targetYaw = -degrees;

        // Tuning parameters
        double kP        = 0.01;  // Proportional gain - adjust as needed
        double maxPower  = 0.4;   // Max motor power for turning
        double minPower  = 0.12;  // Minimum power to overcome friction
        double tolerance = 2.0;   // Acceptable error in degrees

        while (true) {
            double heading = getHeadingDegrees();
            double error   = targetYaw - heading;  // how far we still need to rotate

            // Exit when we're close enough
            if (Math.abs(error) <= tolerance) {
                break;
            }

            // Simple proportional control
            double turnPower = kP * error;

            // Clamp power
            if (turnPower >  maxPower) turnPower =  maxPower;
            if (turnPower < -maxPower) turnPower = -maxPower;

            // Enforce a minimum power so robot doesn't stall
            if (Math.abs(turnPower) < minPower) {
                turnPower = Math.copySign(minPower, turnPower);
            }

            // IMPORTANT: Use your existing rotate helpers.
            // turnPower > 0  => need a CCW turn   -> use rotateLeft
            // turnPower < 0  => need a CW turn    -> use rotateRight
            if (turnPower > 0) {
                rotateLeft(turnPower);
            } else {
                rotateRight(Math.abs(turnPower));
            }

            // Optional telemetry for debugging during tests
            if (telemetry != null) {
                telemetry.addData("Turn Target Yaw", targetYaw);
                telemetry.addData("Heading", heading);
                telemetry.addData("Error", error);
                telemetry.addData("Turn Power", turnPower);

            }
        }

        // Stop motors at the end
        allStop();
    }


}
