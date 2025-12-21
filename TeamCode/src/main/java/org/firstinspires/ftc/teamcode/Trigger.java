package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Trigger {

    // Servo reference
    private Servo trigger;

    // Name must match your Robot Configuration
    private static final String SERVO_NAME = "trigger";

    // HOME and FIRE positions
    // Adjust these based on what you saw in the sweep test.
    // Here: HOME at 1.0, FIRE a quarter-sweep to the left (0.75).
    private static final double HOME_POS = 1.0;
    private static final double FIRE_POS = 0.35  ;

    // How long to stay at FIRE before returning (ms)
    private static final long DWELL_MS = 400;   // tune 150–400

    /** Map the servo and go to HOME. */
    public void init(HardwareMap hw) {
        trigger = hw.get(Servo.class, SERVO_NAME);
        trigger.setPosition(HOME_POS);
    }

    /**
     * One simple fire cycle:
     * HOME -> FIRE (small move) -> wait -> HOME.
     * This is blocking; call from TeleOp when B is pressed.
     */
    public void fire(LinearOpMode opMode) {
        if (trigger == null || opMode == null) return;

        // Move to FIRE
        trigger.setPosition(FIRE_POS);
        opMode.sleep(DWELL_MS);

        // Back to HOME
        trigger.setPosition(HOME_POS);
    }

    /** For telemetry. */
    public double getPosition() {
        return (trigger != null) ? trigger.getPosition() : -1.0;
    }
}
