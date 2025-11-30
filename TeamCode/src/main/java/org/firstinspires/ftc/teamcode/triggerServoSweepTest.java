package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TriggerServoButtonTest", group = "Test")
public class triggerServoSweepTest extends LinearOpMode {

    // Must match the servo name in your Robot Configuration
    private static final String SERVO_NAME = "trigger";

    // Full sweep range (same idea as before)
    private static final double MIN_POS = 0.0;
    private static final double MAX_POS = 1.0;

    // Move only 1/4 of that full range
    private static final double HOME_POS = MIN_POS;
    private static final double FIRE_POS = MIN_POS + 0.25 * (MAX_POS - MIN_POS);

    // How smooth/slow the movement is
    private static final double STEP = 0.01;   // smaller = smoother, slower
    private static final long   STEP_MS = 20;  // delay between steps

    private Servo trigger;

    @Override
    public void runOpMode() {
        trigger = hardwareMap.get(Servo.class, SERVO_NAME);

        // Start at home
        trigger.setPosition(HOME_POS);

        telemetry.addData("Status", "Ready - press gamepad2.b to fire");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        boolean prevB = false;

        while (opModeIsActive()) {
            boolean b = gamepad2.b;

            // Show current servo position + button
            telemetry.addData("gamepad2.b", b);
            telemetry.addData("Servo Position", "%.3f", trigger.getPosition());

            // On rising edge of B: one sweep HOME -> FIRE -> HOME
            if (b && !prevB) {
                telemetry.addLine("B pressed: sweeping servo");
                telemetry.update();

                // Move from HOME to FIRE
                for (double p = HOME_POS; p <= FIRE_POS && opModeIsActive(); p += STEP) {
                    trigger.setPosition(p);
                    telemetry.addData("Servo Position", "%.3f", p);
                    telemetry.update();
                    sleep(STEP_MS);
                }

                // Move from FIRE back to HOME
                for (double p = FIRE_POS; p >= HOME_POS && opModeIsActive(); p -= STEP) {
                    trigger.setPosition(p);
                    telemetry.addData("Servo Position", "%.3f", p);
                    telemetry.update();
                    sleep(STEP_MS);
                }
            }

            prevB = b;

            // Small delay so we’re not spamming the loop
            sleep(10);
        }
    }
}
