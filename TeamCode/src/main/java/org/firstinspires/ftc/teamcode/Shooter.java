package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class Shooter {
    private DcMotorEx left, right;

    private static final double TICKS_PER_REV = 537.7; // example

    public void init(HardwareMap hw) {
        left  = hw.get(DcMotorEx.class, "left_shooter");
        right = hw.get(DcMotorEx.class, "right_shooter");

        left.setDirection(DcMotorSimple.Direction.FORWARD);
        right.setDirection(DcMotorSimple.Direction.REVERSE);

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void shoot(double power) {
        power = Range.clip(power, -1.0, 1.0);
        if (left != null)  left.setPower(power);
        if (right != null) right.setPower(power);
    }

    public void stop() {
        if (left != null)  left.setPower(0);
        if (right != null) right.setPower(0);
    }

    // RPM helpers
    private double ticksPerSecToRpm(double tps) {
        return (tps / TICKS_PER_REV) * 60.0;
    }

    public double getLeftRpm() {
        if (left == null) return 0;
        return ticksPerSecToRpm(left.getVelocity());
    }

    public double getRightRpm() {
        if (right == null) return 0;
        return ticksPerSecToRpm(right.getVelocity());
    }

    public double getAverageRpm() {
        return (Math.abs(getLeftRpm()) + Math.abs(getRightRpm())) / 2.0;
    }
}
