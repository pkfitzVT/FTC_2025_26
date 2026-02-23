package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Blinkin LED subsystem using REV Blinkin LED Driver device.
 * Hardware config name: "led"
 *
 * States:
 *  - Idle: PINK
 *  - Spinning: YELLOW
 *  - Ready: GREEN
 */
public class BlinkinSubsystem {

    private RevBlinkinLedDriver blinkin;

    public void init(HardwareMap hw, String name) {
        blinkin = hw.get(RevBlinkinLedDriver.class, name);
        setIdle();
    }

    public void setIdle() {
        set(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
    }

    public void setSpinning() {
        set(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
    }

    public void setReady() {
        set(RevBlinkinLedDriver.BlinkinPattern.GREEN);
    }

    public void setError() {
        set(RevBlinkinLedDriver.BlinkinPattern.RED);
    }

    private void set(RevBlinkinLedDriver.BlinkinPattern pattern) {
        if (blinkin != null) {
            blinkin.setPattern(pattern);
        }
    }
}