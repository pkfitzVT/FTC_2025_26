package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class ShooterV1 {
    private DcMotorEx left, right;

    // IMPORTANT: This must match your motor+encoder+gear ratio *as returned by getVelocity()*
    // If your RPM readings seem "too low", this constant is likely wrong.
    private static final double TICKS_PER_REV = 537.7; // keep for now since you're using it already

    private double targetRpm = 0;

    public void init(HardwareMap hw) {
        left  = hw.get(DcMotorEx.class, "left_shooter");
        right = hw.get(DcMotorEx.class, "right_shooter");

        left.setDirection(DcMotorSimple.Direction.FORWARD);
        right.setDirection(DcMotorSimple.Direction.REVERSE);

        // For flywheels, FLOAT is often smoother than BRAKE when you turn it off.
        // BRAKE is not "wrong", but it can create abrupt stops and extra current spikes.
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Open-loop (what you currently do)
    public void shoot(double power) {
        targetRpm = 0; // leaving closed-loop mode
        power = Range.clip(power, -1.0, 1.0);
        if (left != null)  left.setPower(power);
        if (right != null) right.setPower(power);
    }

    // Closed-loop: hold a target RPM (recommended for consistent shots)
    public void setTargetRpm(double rpm) {
        targetRpm = Math.max(0, rpm);

        double targetTicksPerSec = rpmToTicksPerSec(targetRpm);

        if (left != null)  left.setVelocity(targetTicksPerSec);
        if (right != null) right.setVelocity(targetTicksPerSec);
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    public void stop() {
        targetRpm = 0;
        if (left != null)  left.setPower(0);
        if (right != null) right.setPower(0);
    }

    // Ready check for feeding
    public boolean isAtSpeed(double toleranceRpm) {
        if (targetRpm <= 0) return false;

        double avg = getAverageRpm();
        return Math.abs(avg - targetRpm) <= toleranceRpm;
    }

    // --- RPM helpers ---
    private double ticksPerSecToRpm(double tps) {
        return (tps / TICKS_PER_REV) * 60.0;
    }

    private double rpmToTicksPerSec(double rpm) {
        return (rpm / 60.0) * TICKS_PER_REV;
    }

    public double getLeftRpm() {
        if (left == null) return 0;
        return ticksPerSecToRpm(Math.abs(left.getVelocity()));
    }

    public double getRightRpm() {
        if (right == null) return 0;
        return ticksPerSecToRpm(Math.abs(right.getVelocity()));
    }

    public double getAverageRpm() {
        return (getLeftRpm() + getRightRpm()) / 2.0;
    }
}
