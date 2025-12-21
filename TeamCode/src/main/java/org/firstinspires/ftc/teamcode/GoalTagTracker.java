package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

/**
 * GoalTagTracker
 *
 * Pure "interpreter" class:
 *  - Does NOT own the camera
 *  - Does NOT build a VisionPortal
 *  - Reads AprilTagDetection list each loop
 *  - Tracks the goal tag (e.g., Blue Goal ID 20)
 *
 * Safe to run alongside DecodeObeliskVision since it shares detections.
 */
public class GoalTagTracker {

    // Change these as needed (add red goal later, etc.)
    private final int goalId;

    private boolean goalVisible = false;
    private Double rangeIn = null;
    private Double bearingDeg = null;

    public GoalTagTracker(int goalId) {
        this.goalId = goalId;
    }

    /**
     * Call once per loop. Pass in the AprilTag detections from your vision manager.
     */
    public void update(List<AprilTagDetection> detections) {
        goalVisible = false;
        rangeIn = null;
        bearingDeg = null;

        if (detections == null) return;

        for (AprilTagDetection d : detections) {
            if (d == null) continue;

            if (d.id == goalId) {
                goalVisible = true;

                // Pose might be null if tag is too small/blurred/etc.
                if (d.ftcPose != null) {
                    rangeIn = d.ftcPose.range;       // inches if processor is set to INCH
                    bearingDeg = d.ftcPose.bearing;  // degrees
                }

                break; // Found our goal tag
            }
        }
    }

    public boolean isGoalVisible() {
        return goalVisible;
    }

    public Double getRangeInches() {
        return rangeIn;
    }

    public Double getBearingDeg() {
        return bearingDeg;
    }

    /**
     * Convenience check for your driver aid / LED:
     * e.g., within 30 inches and bearing -10 to +10 degrees
     */
    public boolean isInRange(double maxRangeIn, double minBearingDeg, double maxBearingDeg) {
        if (rangeIn == null || bearingDeg == null) return false;
        return (rangeIn <= maxRangeIn && bearingDeg >= minBearingDeg && bearingDeg <= maxBearingDeg);
    }

    public int getGoalId() {
        return goalId;
    }
}
