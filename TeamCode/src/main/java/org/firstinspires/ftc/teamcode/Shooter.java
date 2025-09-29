package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class Shooter {
    private DcMotorEx left, right;

    public void init(HardwareMap hw) {
        left  = hw.get(DcMotorEx.class, "left_shooter");
        right = hw.get(DcMotorEx.class, "right_shooter");

        // Set opposite directions so the same +power spins opposite ways
        left.setDirection(DcMotorSimple.Direction.FORWARD);
        right.setDirection(DcMotorSimple.Direction.REVERSE);

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /** Spin both wheels; +power in [0..1] */
    public void shoot(double power) {
        power = Range.clip(power, -1.0, 1.0);
        left.setPower(power);
        right.setPower(power);
    }

    /** Stop both wheels */
    public void stop() {
        left.setPower(0);
        right.setPower(0);
    }
}
