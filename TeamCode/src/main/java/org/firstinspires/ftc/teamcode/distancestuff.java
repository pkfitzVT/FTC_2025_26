package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;

public class distancestuff {

    private DistanceSensor sensorDistance;
    private Rev2mDistanceSensor sensorTimeOfFlight;

    // ---------- Filter settings ----------
    // Median filter window size (odd number recommended)
    private static final int N = 5;

    // Ignore readings outside these practical bounds (REV 2m ~ 0.5" to ~78.7")
    private static final double MIN_VALID_IN = 0.5;
    private static final double MAX_VALID_IN = 85.0;

    // Ring buffer for last N valid readings
    private final double[] window = new double[N];
    private int count = 0;
    private int idx = 0;

    // Diagnostics
    private double lastRawIn = Double.NaN;

    public void init(HardwareMap hardwareMap, String configName) {
        sensorDistance = hardwareMap.get(DistanceSensor.class, configName);

        // Optional REV-only features (timeout flag)
        try {
            sensorTimeOfFlight = (Rev2mDistanceSensor) sensorDistance;
        } catch (Exception e) {
            sensorTimeOfFlight = null;
        }

        Arrays.fill(window, Double.NaN);
        count = 0;
        idx = 0;
        lastRawIn = Double.NaN;
    }

    /**
     * Stable distance in inches.
     * - Reads raw every call
     * - Rejects invalid/out-of-range/timeout
     * - Feeds a median-of-5 filter
     * - Returns the median (stable) once we have samples
     */
    public double getDistanceInches() {
        if (sensorDistance == null) return Double.NaN;

        double raw = sensorDistance.getDistance(DistanceUnit.INCH);
        lastRawIn = raw;

        // Reject bad raw values
        if (Double.isNaN(raw) || Double.isInfinite(raw)) {
            return currentMedianOrNaN();
        }
        if (raw < MIN_VALID_IN || raw > MAX_VALID_IN) {
            return currentMedianOrNaN();
        }

        // If REV API available, reject timeouts (this is the “sample” gold signal)
        if (sensorTimeOfFlight != null && sensorTimeOfFlight.didTimeoutOccur()) {
            return currentMedianOrNaN();
        }

        // Accept sample into ring buffer
        window[idx] = raw;
        idx = (idx + 1) % N;
        if (count < N) count++;

        return medianOfWindow();
    }

    /** Raw (unfiltered) last reading, for debugging */
    public double getLastRawInches() {
        return lastRawIn;
    }

    /** True if the REV sensor reports timeout (if available) */
    public boolean didTimeoutOccur() {
        return sensorTimeOfFlight != null && sensorTimeOfFlight.didTimeoutOccur();
    }

    /** Telemetry helper */
    public String getTelemetryLine() {
        double stable = getDistanceInches();
        double raw = getLastRawInches();
        boolean timeout = didTimeoutOccur();

        String rawStr = Double.isNaN(raw) ? "—" : String.format("%.1f", raw);
        String stStr  = Double.isNaN(stable) ? "—" : String.format("%.1f", stable);

        return String.format("ToF raw=%s in | stable=%s in | timeout=%s",
                rawStr, stStr, timeout);
    }

    // ---------- Helpers ----------

    private double currentMedianOrNaN() {
        return (count == 0) ? Double.NaN : medianOfWindow();
    }

    private double medianOfWindow() {
        // Copy valid portion
        double[] vals = new double[count];
        int j = 0;

        for (double v : window) {
            if (!Double.isNaN(v)) vals[j++] = v;
        }
        if (j == 0) return Double.NaN;

        Arrays.sort(vals, 0, j);
        int mid = j / 2;

        if (j % 2 == 1) return vals[mid];
        return (vals[mid - 1] + vals[mid]) / 2.0;
    }
}