package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Shooter {
    private DcMotorEx left, right;

    // Trigger servo
    private Servo trigger;

    // Keep your tuned setpoints; reversing direction will flip the physical motion
    private static final double TRIGGER_HOME    = 0.65; // retracted / safe
    private static final double TRIGGER_PUSH_90 = 0.15; // ~90° from HOME

    // Dwell time at push (ms)
    private static final long TRIGGER_DWELL_MS = 250;   // tune 150–400 ms

    public void init(HardwareMap hw) {
        // Motors
        left  = hw.get(DcMotorEx.class, "left_shooter");
        right = hw.get(DcMotorEx.class, "right_shooter");

        left.setDirection(DcMotorSimple.Direction.FORWARD);
        right.setDirection(DcMotorSimple.Direction.REVERSE);

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Servo
        trigger = hw.get(Servo.class, "trigger");

        // ✅ Reverse the servo's direction so movement is opposite
        trigger.setDirection(Servo.Direction.FORWARD);

        trigger.setPosition(TRIGGER_HOME);
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

    // ---------- Trigger helpers ----------

    /** Move immediately to the retracted (home) position. */
    public void triggerHome() {
        if (trigger != null) trigger.setPosition(TRIGGER_HOME);
    }

    /** Move immediately to the ~90° push position. */
    public void triggerPush90() {
        if (trigger != null) trigger.setPosition(TRIGGER_PUSH_90);
    }

    /**
     * One-button cycle: rotate ~90°, dwell briefly, return to home.
     * Blocking for ~TRIGGER_DWELL_MS; call from a LinearOpMode (e.g., in TeleOp on B press).
     */
    public void fireTrigger(LinearOpMode opMode) {
        if (trigger == null || opMode == null) return;

        // PUSH → dwell → HOME
        trigger.setPosition(TRIGGER_PUSH_90);
        opMode.sleep(TRIGGER_DWELL_MS);
        trigger.setPosition(TRIGGER_HOME);
        // Optional: opMode.sleep(150); // settle time
    }

    /** For telemetry if you want it. */
    public double getTriggerPosition() {
        return (trigger != null) ? trigger.getPosition() : -1.0;
    }
}
