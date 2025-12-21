package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


import java.util.List;

/**
 * DecodeObeliskVision
 *
 * Standalone helper class (NOT an OpMode) that:
 *  - Creates an AprilTagProcessor + VisionPortal
 *  - Looks for the OBELISK tag (IDs 21, 22, 23 in DECODE)
 *  - Interprets that ID as a MOTIF (GPP, PGP, or PPG)
 *  - Exposes simple getters so TeleOp / Auto know which pattern to build
 *
 * Usage from TeleOp or Auto:
 *  1. Create one instance in your OpMode (as a field).
 *  2. Call init() during your init phase (before waitForStart()).
 *  3. Inside the main loop, call update() once each cycle.
 *  4. Read getCurrentMotif() or getMotifString() to know the pattern.
 *  5. Optionally call addTelemetry() to display info on DS.
 */
public class DecodeObeliskVision {

    // --- Public "enum" for the 3 possible motifs + UNKNOWN ---
    public enum Motif {
        GPP,   // Green, Purple, Purple
        PGP,   // Purple, Green, Purple
        PPG,   // Purple, Purple, Green
        UNKNOWN
    }

    // =======================
    //  Configuration options
    // =======================

    // If you are using a USB webcam plugged into the Control Hub / Robot Controller
    private static final boolean USE_WEBCAM = true;

    // Name of the webcam in the RC configuration (edit to match your config!)
    private static final String WEBCAM_NAME = "Webcam 1";

    // =======================
    //  Internal state
    // =======================

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    // What we care about for the game:
    private Motif currentMotif = Motif.UNKNOWN;  // Latest interpreted motif
    private int currentTagId = -1;               // Raw AprilTag ID (21, 22, 23, or -1)
    private boolean tagVisible = false;          // Did we see a valid OBELISK tag this frame?

    // Optionally keep a reference to telemetry for quick debug printing
    private final Telemetry telemetry;

    /**
     * Constructor.
     *
     * @param telemetry Telemetry from your OpMode (for optional debug output).
     */
    public DecodeObeliskVision(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    /**
     * Call this once from your OpMode's init section to set up the
     * AprilTag processor and VisionPortal.
     *
     * @param hardwareMap OpMode hardwareMap
     */
    public void init(HardwareMap hardwareMap) {

        // -------------------------
        // 1) Build the AprilTag processor
        // -------------------------
        aprilTagProcessor = new AprilTagProcessor.Builder()
                // You can un-comment and adjust these if needed:
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        // -------------------------
        // 2) Build the VisionPortal (camera + processors)
        // -------------------------
        VisionPortal.Builder builder = new VisionPortal.Builder();

        if (USE_WEBCAM) {
            // Use external webcam (Control Hub / RC configuration must define this name)
            builder.setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME));
        } else {
            // Use built-in RC phone camera (if you were using phones)
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Optional: choose resolution (commented means "use default")
        // builder.setCameraResolution(new Size(640, 480));

        // Optional: enable or disable live preview window
        builder.enableLiveView(true);

        // Attach our AprilTag processor
        builder.addProcessor(aprilTagProcessor);

        // Finally, build the VisionPortal; camera will start streaming
        visionPortal = builder.build();
    }

    /**
     * Call this once per loop in your TeleOp / Auto.
     * It reads detections and updates our "currentMotif" field.
     */
    public void update() {
        tagVisible = false;
        currentTagId = -1;
        currentMotif = Motif.UNKNOWN;

        if (aprilTagProcessor == null) {
            // Defensive: if init() was not called, avoid crash.
            return;
        }

        // Get all current detections from the processor.
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

        // Simple approach:
        //  - look through detections
        //  - if we find one with ID 21, 22, or 23, treat that as the OBELISK tag
        //  - pick the first valid one we see
        for (AprilTagDetection detection : detections) {
            int id = detection.id;

            if (id == 21 || id == 22 || id == 23) {
                tagVisible = true;
                currentTagId = id;
                currentMotif = motifFromId(id);
                break;  // Stop after we find one valid OBELISK tag
            }
        }
    }

    /**
     * Helper: map the AprilTag ID to the game motif.
     * According to the DECODE manual, OBELISK tag IDs map:
     *   21 -> GPP
     *   22 -> PGP
     *   23 -> PPG
     */
    private Motif motifFromId(int id) {
        switch (id) {
            case 21:
                return Motif.GPP;
            case 22:
                return Motif.PGP;
            case 23:
                return Motif.PPG;
            default:
                return Motif.UNKNOWN;
        }
    }

    // =======================
    //  Public getters
    // =======================

    /** @return true if we currently see one of the OBELISK tags (21–23). */
    public boolean isTagVisible() {
        return tagVisible;
    }

    /** @return raw AprilTag ID (21, 22, 23) or -1 if none seen. */
    public int getCurrentTagId() {
        return currentTagId;
    }

    /** @return Motif enum for the current reading. */
    public Motif getCurrentMotif() {
        return currentMotif;
    }

    /**
     * @return Motif as a simple 3-character String, for example "GPP".
     *         Useful if you want to pass it around or log it.
     */
    public String getMotifString() {
        switch (currentMotif) {
            case GPP:
                return "GPP";
            case PGP:
                return "PGP";
            case PPG:
                return "PPG";
            default:
                return "UNKNOWN";
        }
    }

    /**
     * Optional: returns the 9-index pattern for the RAMP as an array.
     * The game repeats the 3-color motif three times across indices 1–9.
     * e.g. GPP -> {G, P, P, G, P, P, G, P, P}
     */
    public char[] getRampPattern() {
        char[] result = new char[9];

        String motif = getMotifString(); // e.g. "GPP"
        if (motif.equals("UNKNOWN")) {
            // Fill with '?' if we don't know yet
            for (int i = 0; i < 9; i++) {
                result[i] = '?';
            }
            return result;
        }

        // Repeat motif 3 times
        for (int i = 0; i < 9; i++) {
            result[i] = motif.charAt(i % 3);
        }
        return result;
    }

    /**
     * Add helpful lines to the OpMode telemetry.
     * Call AFTER update() each loop, before telemetry.update().
     */
    public void addTelemetry() {
        if (telemetry == null) return;

        telemetry.addLine("=== OBELISK AprilTag ===");
        telemetry.addData("Tag visible", tagVisible);
        telemetry.addData("Tag ID", currentTagId);
        telemetry.addData("Motif", getMotifString());

        char[] pattern = getRampPattern();
        telemetry.addData("Ramp pattern",
                "%c %c %c  %c %c %c  %c %c %c",
                pattern[0], pattern[1], pattern[2],
                pattern[3], pattern[4], pattern[5],
                pattern[6], pattern[7], pattern[8]);
    }

    public VisionPortal getVisionPortal() {
        return visionPortal;
    }

    public AprilTagDetection getCurrentDetection() {
        if (aprilTagProcessor == null) return null;
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        for (AprilTagDetection detection : detections) {
            int id = detection.id;
            if (id == 21 || id == 22 || id == 23) {
                return detection;
            }
        }
        return null;
    }

    public Double getCurrentRangeInches() {
        AprilTagDetection det = getCurrentDetection();
        if (det == null || det.ftcPose == null) return null;
        return det.ftcPose.range;   // in inches if you set output units
    }

    public Double getCurrentBearingDeg() {
        AprilTagDetection det = getCurrentDetection();
        if (det == null || det.ftcPose == null) return null;
        return det.ftcPose.bearing; // degrees left/right
    }

    public List<AprilTagDetection> getDetections() {
        if (aprilTagProcessor == null) return null;
        return aprilTagProcessor.getDetections();
    }



    /**
     * Stop streaming and free resources.
     * Call this from OpMode when you're totally done with vision.
     */
    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
            visionPortal = null;
        }
    }
}
