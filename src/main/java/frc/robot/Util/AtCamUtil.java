package frc.robot.Util;

import static edu.wpi.first.units.Units.Degrees;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import javax.crypto.spec.OAEPParameterSpec;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AtCamUtil {
    private PhotonCamera cam;
    private AprilTagFieldLayout aprilTagFieldLayout;

    private Transform3d robotToCamera;
    private PhotonPipelineResult result;
    private String name;
    private Transform2d robotToCam2D;
    private PhotonPoseEstimator photonPoseEstimator;
    private Rotation2d rot;
    private double lastTimeStamp = 0;
    public AtCamUtil(String name, Transform3d robotToCamera, Rotation2d rot) {
        // constructing the camera
        this.name = name;
        // this.cam = new PhotonCamera(NetworkTableInstance.getDefault(), name);
        this.cam = new PhotonCamera(name);
        this.cam.setPipelineIndex(1);
        this.robotToCamera = robotToCamera;
        this.robotToCam2D = new Transform2d(this.robotToCamera.getX(), this.robotToCamera.getY(),
                this.robotToCamera.getRotation().toRotation2d());
        this.rot = rot;
        this.cam.takeOutputSnapshot();
        boolean fieldWork;
        try {
            this.aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
            fieldWork = true;
        } catch (Exception e) {
            this.aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
            fieldWork = false;
        }
        SmartDashboard.putBoolean("filed works?", fieldWork);
        this.photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, robotToCamera);
        this.result = new PhotonPipelineResult();
    }

    public boolean isConnected() {
        return this.cam.isConnected();
    }

    public String getName() {
        return this.name;
    }

    // TODO: remove this function
    private boolean hasTarget() {
        // System.out.println("[target] hasTarget " + this.result.hasTargets());
        SmartDashboard.putBoolean(name + "/result has targets?", this.result.hasTargets());
        return this.result == null ? false : this.result.hasTargets();
    }

    public boolean hasValidTarget(double distance) {
        // System.out.println("[target] in hasValidTarget");

        boolean distance_ok = distance < 2;
        boolean has_target = hasTarget();
        // System.out.println("[target] " + distance_ok + " " + has_target);

        // return has_target && distance_ok;
        return has_target;
    }

    // public void updateResult() {
    // // TODO: Need to take the latest by sequence id
    // List<PhotonPipelineResult> unreadResults = this.cam.getAllUnreadResults();
    // // System.out.println("[unread_results] " + unreadResults.toString());

    // if (!unreadResults.isEmpty()) {
    // // TODO: this should be the first element
    // PhotonPipelineResult newResult = null;
    // PhotonPipelineResult nonEmptyMTR = null;
    // long maxId = -1;
    // for(PhotonPipelineResult result: unreadResults){
    // long resultId = result.metadata.sequenceID;
    // if (resultId > maxId) {
    // newResult = result;
    // maxId = resultId;
    // }

    // if(!newResult.getMultiTagResult().isEmpty()) {
    // System.out.println(" [MTR] OMG we found it!");
    // nonEmptyMTR = result;
    // }
    // }

    // Optional<MultiTargetPNPResult> newMultiTagResult =
    // newResult.getMultiTagResult();
    // System.out.println(" [multitagConditions] " + !newMultiTagResult.isEmpty() +
    // " " + newResult.hasTargets());
    // if (!newMultiTagResult.isEmpty() && newResult.hasTargets()) {
    // this.result = newResult;
    // SmartDashboard.putString(name + "target info", newResult.toString());
    // }
    // }
    // }

    public Optional<Pose3d> getTagPose3d(int ID) {
        Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(ID);
        return tagPose;
    }

    public int getID() {
        // System.out.println("[getId] in here");
        if (hasTarget()) {
            // System.out.println("[updateResults] found targets");
            SmartDashboard.putNumber(name + "/tag ID", this.result.getBestTarget().fiducialId);
            return this.result.getBestTarget().getFiducialId();
        }
        SmartDashboard.putNumber(name + "/tag ID", -1);
        return -1;
    }

    public Optional<Double> getTargetX() {
        Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(getID());

        if (!hasTarget() || tagPose.isEmpty()) {
            return Optional.empty();
        }

        // Get the transformation from the camera to the target
        Transform3d camToTarget = result.getBestTarget().getBestCameraToTarget();

        // Compute the X position of the target relative to the camera
        double targetX = camToTarget.getX();

        return Optional.of(targetX);
    }

    public Optional<Double> getTargetY() {
        Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(getID());

        if (!hasTarget() || tagPose.isEmpty()) {
            return Optional.empty();
        }

        // Get the transformation from the camera to the target
        Transform3d camToTarget = result.getBestTarget().getBestCameraToTarget();

        // Compute the X position of the target relative to the camera
        double targetY = camToTarget.getY();

        return Optional.of(targetY);
    }

    public Optional<Angle> getTargetYaw() {
        if (hasTarget()) {
            return Optional.of(Degrees.of(result.getBestTarget().getYaw()));
        }
        return Optional.empty();
    }

    public double distanceFromTargetMeters() {
        // SmartDashboard.putNumber("cameraHeightMeters", this.robotToCamera.getZ());
        // SmartDashboard.putNumber("targetHeightMeters",
        // this.aprilTagFieldLayout.getTagPose(this.getID()).get().getZ());
        // SmartDashboard.putNumber("cameraPitchRadians",
        // this.robotToCamera.getRotation().getY());
        // SmartDashboard.putNumber("targetPitchRadians",
        // this.aprilTagFieldLayout.getTagPose(getID()).get().getY());

        // System.out.println("[distanceFromTargetMeters] in here");
        if (this.getID() == -1) {
            // System.out.println("[distanceFromTargetMeters] didn't find IDs");
            return 1.2;
        }
        // System.out.println(result.getBestTarget().getBestCameraToTarget().getTranslation().getNorm());
        return result.getBestTarget().getBestCameraToTarget().getTranslation().getNorm();
        // return PhotonUtils.calculateDistanceToTargetMeters(
        // this.robotToCamera.getZ(), // Meters
        // this.aprilTagFieldLayout.getTagPose(this.getID()).get().getZ(), // Meters
        // this.robotToCamera.getRotation().getY(), // Radians
        // Math.toRadians(this.result.getBestTarget().pitch));

    }
    // public double distanceFromTargetMeters() {

    // var target = this.result.getBestTarget();

    // var cameraToTarget = target.getBestCameraToTarget();

    // double range = cameraToTarget.getX();

    // return cameraToTarget.getTranslation().getNorm();

    // }

    // public Pose2d getPoseFromCamera() {
    // if (hasValidTarget()) {
    // PhotonTrackedTarget bestTarget = this.result.getBestTarget();

    // Optional<Pose3d> tagPose3D =
    // aprilTagFieldLayout.getTagPose(bestTarget.getFiducialId());

    // Transform3d cameraToTarget3d = bestTarget.getBestCameraToTarget();
    // Translation3d translation3d = cameraToTarget3d.getTranslation();
    // Translation2d cameraToTargetTranslation = translation3d.toTranslation2d();

    // Transform2d CamToTarget =
    // PhotonUtils.estimateCameraToTarget(cameraToTargetTranslation,
    // tagPose3D.get().toPose2d(), this.rot);

    // return PhotonUtils.estimateFieldToRobot(CamToTarget,
    // this.aprilTagFieldLayout.getTagPose(this.getID()).get().toPose2d(),
    // this.robotToCam2D.times(-1)).plus(this.robotToCam2D);
    // // return PhotonUtils.estimateFieldToRobot(CamToTarget,
    // // tagPose3D.get().toPose2d(),
    // // new Transform2d(this.robotToCamera.getX(), this.robotToCamera.getY(),
    // // this.robotToCamera.getRotation().toRotation2d()));
    // }

    // return new Pose2d(); // Return an empty pose if no valid target
    // }
    public Pose2d getPoseFromCamera(Pose2d currentPose) {
        // if (this.photonPoseEstimator.estimateCoprocMultiTagPose(result).isEmpty()) {
        // return
        // this.photonPoseEstimator.estimateClosestToCameraHeightPose(result).get().estimatedPose.toPose2d();
        // }
        // return
        // this.photonPoseEstimator.estimateCoprocMultiTagPose(result).get().estimatedPose.toPose2d();
        Pose2d result = currentPose.plus(
                to2dTransform(this.result.getBestTarget().bestCameraToTarget));
        SmartDashboard.putNumber(name + "/yawAng", this.result.getBestTarget().bestCameraToTarget.getRotation().getZ());
        return result;

    }

    public double getCameraTimeStampSec() {
        return this.lastTimeStamp;
    }

    private Transform2d to2dTransform(Transform3d estimatedPose) {
        return new Transform2d(
                new Translation2d(estimatedPose.getX(), estimatedPose.getZ()),
                estimatedPose.getRotation().toRotation2d());
    }

    // --------------------------------------------------------------------------------------
    public Pose2d updateResult(Pose2d currentPose) {
        var results = this.cam.getAllUnreadResults();
        var estimatedPose = new Pose2d();
        if (results == null) {
            SmartDashboard.putString(name + "/status", "result is null");
            return estimatedPose;
        }
        for (var result : results) {
            PhotonTrackedTarget defaultTarget;
            Transform3d bestTarget3d;
            Transform3d altTarget3d;
            int fiducialId;

            SmartDashboard.putBoolean("multitag exist",
                    result.multitagResult.isPresent());
            if (result.multitagResult.isPresent()) {
                this.result = result;
                SmartDashboard.putString(this.name + "/multitag ", "true");
                bestTarget3d = result.multitagResult.get().estimatedPose.best;
                altTarget3d = result.multitagResult.get().estimatedPose.alt;

                SmartDashboard.putString(this.name + "/update result pose",
                        translation3dToPose2d(bestTarget3d).toString());
                SmartDashboard.putString(name + "/status", "multi tag is real");

                fiducialId = result.multitagResult.get().fiducialIDsUsed.get(0) < 0.2
                        ? result.multitagResult.get().fiducialIDsUsed.get(0)
                        : result.multitagResult.get().fiducialIDsUsed.get(1);
                // return
                // translation3dToPose2d(bestTarget3d.plus(this.robotToCamera.inverse()));

            } else {
                SmartDashboard.putString(name + "/status", "multi tag isn't real");
                defaultTarget = result.getBestTarget();
                if (defaultTarget == null)
                    continue;
                SmartDashboard.putString(this.name + "/best target",
                        defaultTarget.toString());
                this.result = result;
                bestTarget3d = defaultTarget.bestCameraToTarget;
                altTarget3d = defaultTarget.altCameraToTarget;
                fiducialId = defaultTarget.fiducialId;

                if (aprilTagFieldLayout.getTagPose(fiducialId).isPresent()) {
                    Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
                            result.getBestTarget().getBestCameraToTarget(),
                            aprilTagFieldLayout.getTagPose(fiducialId).get(), this.robotToCamera);
                    SmartDashboard.putString(name + "/photon utils pose", robotPose.toString());
                    // return robotPose.toPose2d();
                }

            }

            double currentYaw = currentPose.getRotation().getDegrees();

            double bestYawError = Math.abs(MathUtil.inputModulus(
                    bestTarget3d.getRotation().getZ() - currentYaw,
                    -180.0,
                    180.0));
            SmartDashboard.putString(this.name + "/best cam ", bestTarget3d.toString());

            double alternateYawError = Math.abs(MathUtil.inputModulus(
                    altTarget3d.getRotation().getZ() - currentYaw,
                    -180.0,
                    180.0));
            SmartDashboard.putString(this.name + "/alternate cam",
                    altTarget3d.toString());

            var bestTransform = bestYawError < alternateYawError ? bestTarget3d : altTarget3d;

            Pose3d tagPose = aprilTagFieldLayout.getTagPose(fiducialId).get();

            estimatedPose = tagPose.transformBy(bestTransform.inverse()).transformBy(robotToCamera.inverse())
                    .toPose2d();
        }

        Optional<EstimatedRobotPose> ePose = photonPoseEstimator.estimateCoprocMultiTagPose(result);
        if (ePose.isEmpty()) {
            this.lastTimeStamp = photonPoseEstimator.estimateLowestAmbiguityPose(result).get().timestampSeconds;
            return photonPoseEstimator.estimateLowestAmbiguityPose(result).get().estimatedPose.toPose2d();
        }
        this.lastTimeStamp = ePose.get().timestampSeconds;
        return ePose.get().estimatedPose.toPose2d();
    }

    private Pose2d translation3dToPose2d(Transform3d transform3d) {

        return new Pose2d(transform3d.getTranslation().toTranslation2d(), transform3d.getRotation().toRotation2d());
    }

    // public Pose2d updateResult() {

    // var results = this.cam.getAllUnreadResults();
    // var estimatedPose = new Pose2d();
    // if (results == null) {
    // SmartDashboard.putString(name + "/status", "result is null");
    // return estimatedPose;
    // }
    // for (var result : results) {
    // PhotonTrackedTarget defaultTarget;
    // Transform3d bestTarget3d;
    // Transform3d altTarget3d;
    // int fiducialId;

    // SmartDashboard.putBoolean("multitag exist",
    // result.multitagResult.isPresent());
    // if (result.multitagResult.isPresent()) {
    // this.result = result;
    // SmartDashboard.putString(this.name + "/multitag ", "true");
    // bestTarget3d = result.multitagResult.get().estimatedPose.best;
    // altTarget3d = result.multitagResult.get().estimatedPose.alt;

    // SmartDashboard.putString(this.name + "/update result pose",
    // translation3dToPose2d(bestTarget3d).toString());
    // SmartDashboard.putString(name + "/status", "multi tag is real");

    // fiducialId = result.multitagResult.get().fiducialIDsUsed.get(0) < 0.2
    // ? result.multitagResult.get().fiducialIDsUsed.get(0)
    // : result.multitagResult.get().fiducialIDsUsed.get(1);

    // } else {
    // SmartDashboard.putString(name + "/status", "multi tag isn't real");
    // defaultTarget = result.getBestTarget();
    // if (defaultTarget == null)
    // continue;
    // SmartDashboard.putString(this.name + "/best target",
    // defaultTarget.toString());
    // this.result = result;
    // bestTarget3d = defaultTarget.bestCameraToTarget;
    // altTarget3d = defaultTarget.altCameraToTarget;
    // fiducialId = defaultTarget.fiducialId;

    // }

    // SmartDashboard.putString(this.name + "/best cam ", bestTarget3d.toString());
    // }

    // Optional<EstimatedRobotPose> ePose =
    // photonPoseEstimator.estimateCoprocMultiTagPose(result);
    // if (ePose.isEmpty()) {
    // return
    // photonPoseEstimator.estimateLowestAmbiguityPose(result).get().estimatedPose.toPose2d();
    // }
    // return ePose.get().estimatedPose.toPose2d();
    // // return estimatedPose;
    // }
}
