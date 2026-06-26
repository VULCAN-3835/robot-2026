// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

public class LimelightUtil {

    /** Holds a single parsed pose estimate from the Limelight. */
    public static class PoseEstimate {
        public final Pose2d pose;
        public final double timestampSeconds;
        public final int tagCount;
        public final double avgTagDist;

        public PoseEstimate(Pose2d pose, double timestampSeconds, int tagCount, double avgTagDist) {
            this.pose             = pose;
            this.timestampSeconds = timestampSeconds;
            this.tagCount         = tagCount;
            this.avgTagDist       = avgTagDist;
        }
    }

    private final NetworkTable limelight;
    private double lastProcessedLatencyMs = -1;

    /** Creates a new LimelightUtil. */
    public LimelightUtil(String limelightName) {
        this.limelight = NetworkTableInstance.getDefault().getTable(limelightName);
    }

    // ── Basic target info ────────────────────────────────────────────────────

    /** Horizontal offset from crosshair to target (degrees). */
    public double getX() {
        return limelight.getEntry("tx").getDouble(0);
    }

    /** Vertical offset from crosshair to target (degrees). */
    public double getY() {
        return limelight.getEntry("ty").getDouble(0);
    }

    /** Target area as percent of image (0–100). */
    public double getA() {
        return limelight.getEntry("ta").getDouble(0);
    }

    /** True when the Limelight sees at least one valid target. */
    public boolean cameraHasTarget() {
        return limelight.getEntry("tv").getDouble(0) != 0;
    }

    /** ID of the primary AprilTag being tracked (-1 = none). */
    public double getAprilTagID() {
        return limelight.getEntry("tid").getDouble(0);
    }

    /** Yaw of the target in camera space (degrees). */
    public double getTargetYaw() {
        double[] pose = limelight.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);
        return pose[4];
    }

    // ── Pipeline control ─────────────────────────────────────────────────────

    public void setAprilMode()     { limelight.getEntry("pipeline").setNumber(0); }
    public void setReflectorMode() { limelight.getEntry("pipeline").setNumber(1); }
    public void setPipeline(int p) { limelight.getEntry("pipeline").setNumber(p); }

    // ── Distance ─────────────────────────────────────────────────────────────

    /** 3-D distance from camera to primary target (meters). */
    public double distanceFromTargetMeters() {
        double[] t = limelight.getEntry("botpose_targetspace").getDoubleArray(new double[6]);
        return Math.sqrt(t[0]*t[0] + t[1]*t[1] + t[2]*t[2]);
    }

    /** True if a target is visible and within 3 m. */
    public boolean hasValidTarget() {
        return cameraHasTarget() && distanceFromTargetMeters() < 3;
    }

    // ── Pose estimation (MegaTag 1 — field-relative, WPIBlue frame) ──────────

    /**
     * Sends the robot's current yaw to the Limelight each loop.
     * Required for MegaTag 2; harmless when using MegaTag 1.
     */
    public void setRobotOrientation(double yawDeg) {
        limelight.getEntry("robot_orientation_set")
                 .setDoubleArray(new double[]{yawDeg, 0, 0, 0, 0, 0});
    }

    /**
     * Returns a PoseEstimate from MegaTag 1 (botpose_wpiblue), or null if:
     *   - no valid data is available
     *   - the measurement hasn't changed since last call (stale dedup)
     *   - the position is at (0, 0) — Limelight default when no fix
     */
    public PoseEstimate getPoseEstimate() {
        double[] raw = limelight.getEntry("botpose_wpiblue").getDoubleArray(new double[0]);

        // Need at least x, y, z, roll, pitch, yaw, latency
        if (raw.length < 7) return null;

        // (0,0) is the Limelight "no data" sentinel
        if (raw[0] == 0.0 && raw[1] == 0.0) return null;

        double latencyMs = raw[6];

        // Skip if same frame (Limelight ~10–20 Hz, robot 50 Hz). Use tolerance not ==
        if (Math.abs(latencyMs - lastProcessedLatencyMs) < 0.001) return null;
        lastProcessedLatencyMs = latencyMs;

        // Tag count: modern firmware (LL3+) returns 11 elements with count at [7]
        int tagCount = raw.length >= 8
                ? (int) raw[7]
                : (int) limelight.getEntry("tv").getDouble(0);

        // Average tag distance: available at [9] in 11-element arrays
        double avgTagDist = raw.length >= 10 ? raw[9] : 0.0;

        Pose2d pose = new Pose2d(raw[0], raw[1], Rotation2d.fromDegrees(raw[5]));
        double timestamp = Timer.getFPGATimestamp() - (latencyMs / 1000.0);

        return new PoseEstimate(pose, timestamp, tagCount, avgTagDist);
    }

    /**
     * Legacy simple pose getter (no latency compensation, no dedup).
     * Prefer getPoseEstimate() for odometry.
     */
    public Pose2d getPoseFromCamera() {
        double[] botpose = limelight.getEntry("botpose_wpiblue").getDoubleArray(new double[0]);
        if (botpose.length < 6) return new Pose2d();
        return new Pose2d(botpose[0], botpose[1], Rotation2d.fromDegrees(botpose[5]));
    }

    /**
     * Legacy timestamp getter — total pipeline + capture latency in seconds.
     * For proper timestamps use getPoseEstimate().timestampSeconds instead.
     */
    public double getCameraTimeStampSec() {
        double cl = limelight.getEntry("cl").getDouble(0);
        double tl = limelight.getEntry("tl").getDouble(0);
        return (cl + tl) / 1000.0;
    }
}
