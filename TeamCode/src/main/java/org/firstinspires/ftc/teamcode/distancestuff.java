package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DistanceSensor;   // ✅ import the DistanceSensor interface
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;  // ✅ import the DistanceUnit enum

public class distancestuff {
    private DistanceSensor distanceSensor;

    /** Initialize distance sensor from hardware map */
    public void init(HardwareMap hardwareMap, String configName) {
        // Make sure the configName matches your RC configuration ("distance_sensor")
        distanceSensor = hardwareMap.get(DistanceSensor.class, configName);
    }

    /** Return distance in inches (returns NaN if invalid or out of range) */
    public double getDistanceInches() {
        if (distanceSensor == null) return Double.NaN;
        return distanceSensor.getDistance(DistanceUnit.INCH);
    }
}