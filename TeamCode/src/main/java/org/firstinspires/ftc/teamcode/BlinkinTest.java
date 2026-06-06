package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Blinkin Cycle Test", group="Test")
public class BlinkinTest extends OpMode {
    private RevBlinkinLedDriver blinkin;
    private int state = 0;
    private long lastMs = 0;

    @Override
    public void init() {
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "led");
        lastMs = System.currentTimeMillis();
    }

    @Override
    public void loop() {
        long now = System.currentTimeMillis();
        if (now - lastMs > 800) { // change every 0.8s
            state = (state + 1) % 4;
            lastMs = now;

            switch (state) {
                case 0:
                    blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                    break;
                case 1:
                    blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                    break;
                case 2:
                    blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                    break;
                default:
                    blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_WHITE);
                    break;
            }
        }
    }
}