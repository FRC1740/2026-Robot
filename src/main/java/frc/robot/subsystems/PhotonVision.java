// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

/**
 * Photonvision abstraction providing the best result and the camera offset for the result
 */
public class PhotonVision extends SubsystemBase {
    /** Creates a new PhotonVision. */
    PhotonCamera cam;
    PhotonCamera cam2;
    PhotonPoseEstimator Cam2PoseEstimator;
    PhotonPoseEstimator Cam1PoseEstimator;
    PhotonTrackedTarget bestTarget;
    public PhotonPipelineResult lastResult;
    String lastCamName;
    CommandSwerveDrivetrain m_drive;
    // QuestNavSubsystem m_quest;
    Pose2d pose = new Pose2d();

    NetworkTable VisionTable = NetworkTableInstance.getDefault().getTable("Vision");
    StructArrayPublisher<Pose2d> Cam1Publisher = VisionTable
            .getStructArrayTopic("Cam1", Pose2d.struct).publish();
    StructArrayPublisher<Pose2d> Cam2Publisher = VisionTable
            .getStructArrayTopic("Cam2", Pose2d.struct).publish();
    StructArrayPublisher<Pose2d> Cam1PublisherPos = VisionTable
            .getStructArrayTopic("Cam1RobotPosition", Pose2d.struct).publish();
    StructArrayPublisher<Pose2d> Cam2PublisherPos = VisionTable
            .getStructArrayTopic("Cam2RobotPosition", Pose2d.struct).publish();

    private static PhotonVision instance;

    public static PhotonVision getInstance() {
        if(instance == null) {
            instance = new PhotonVision();
        }
        return instance;
    }

    public PhotonVision() {
        // m_quest = QuestNavSubsystem.getInstance();
        cam = new PhotonCamera(VisionConstants.camName);
        cam2 = new PhotonCamera(VisionConstants.cam2Name);
        cam.setDriverMode(false);
        cam2.setDriverMode(false);
        m_drive = CommandSwerveDrivetrain.getInstance();

        Cam1PoseEstimator = new PhotonPoseEstimator(
            VisionConstants.aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            VisionConstants.RobotToCam1);
        // TODO! not enabled MULTI_TAG_PNP_ON_COPROCESSOR
        Cam2PoseEstimator = new PhotonPoseEstimator(
            VisionConstants.aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            VisionConstants.RobotToCam2);
    }

    @Override
    public void periodic() {
        Cam1PublisherPos.set(new Pose2d[] {
                new Pose2d(
                    VisionConstants.RobotToCam1.getX() + m_drive.getState().Pose.getX(),
                    VisionConstants.RobotToCam1.getY() + m_drive.getState().Pose.getY(), 
                    new Rotation2d(VisionConstants.RobotToCam1.getRotation().getMeasureAngle()).rotateBy(m_drive.getState().Pose.getRotation()))
            });
        Cam2PublisherPos.set(new Pose2d[] {
                new Pose2d(
                    VisionConstants.RobotToCam2.getX() + m_drive.getState().Pose.getX(),
                    VisionConstants.RobotToCam2.getY() + m_drive.getState().Pose.getY(), 
                    new Rotation2d(VisionConstants.RobotToCam2.getRotation().getMeasureAngle()).rotateBy(m_drive.getState().Pose.getRotation()))
            });
        // This method will be called once per scheduler run
        // Get latest result
        PhotonPipelineResult result = getLatestResult();
        // If it exists, get the best result and apply the measurement to the pose
        if (result != null) {
            if (result.hasTargets()) {
                lastResult = result;
                EstimatedRobotPose estimatedPose = ifExistsGetEstimatedRobotPose();
                // really shouldn't be null but just in case
                if (estimatedPose != null) {
                    Pose2d pose = new Pose2d(
                        estimatedPose.estimatedPose.getX(),
                        estimatedPose.estimatedPose.getY(),
                        estimatedPose.estimatedPose.getRotation().toRotation2d());

                    m_drive.addVisionMeasurement(
                        new Pose2d(
                            pose.getX(),
                            pose.getY(),
                            pose.getRotation()),
                            // m_drive.getState().Pose.getRotation()), // ignore vision rot
                        result.getTimestampSeconds());

                    // // TODO! if disabled, should constantly set pose
                    // if (result.getBestTarget().poseAmbiguity < VisionConstants.questVisionUpdateThreshold) {
                    //     m_quest.setPose(pose);
                    // }

                    // // publish results
                    if (lastCamName == VisionConstants.camName) {
                        Cam1Publisher.set(new Pose2d[] { pose });
                    } else {
                        Cam2Publisher.set(new Pose2d[] { pose });
                    }
                }
            }
        }
    }

    public PhotonPipelineResult getLatestResult() {
        PhotonPipelineResult result = null;

        List<PhotonPipelineResult> resultList = cam.getAllUnreadResults();
        if (!resultList.isEmpty()) {
            result = resultList.get(resultList.size() - 1);
            if (result.hasTargets()) {
                bestTarget = result.getBestTarget();
                if (bestTarget.area > 0.1) {
                    lastCamName = VisionConstants.camName;
                    return result;
                }
            }
        }

        resultList = cam2.getAllUnreadResults();
        if (!resultList.isEmpty()) {
            result = resultList.get(resultList.size() - 1);
            if (result.hasTargets()) {
                bestTarget = result.getBestTarget();
                if (bestTarget.area > 0.1) {
                    lastCamName = VisionConstants.cam2Name;
                    return result;
                }
            }
        }

        return null;
    }

    public Optional<EstimatedRobotPose> getVisionPoseEstimationResult() {
        if (lastResult != null) {
            if (lastResult.hasTargets()) {
                if (lastCamName == "Cam1") {
                    return Cam1PoseEstimator.update(lastResult);
                } else {
                    return Cam2PoseEstimator.update(lastResult);
                }
            }
        }

        return null;
    }

    public EstimatedRobotPose ifExistsGetEstimatedRobotPose() {
        Optional<EstimatedRobotPose> estimatedPose = getVisionPoseEstimationResult();

        if (estimatedPose != null) {
            if (estimatedPose.isPresent()) {
                return estimatedPose.get();
            }
        }
        // System.out.println("null pose");
        return null;
    }

    public PhotonTrackedTarget getBestTarget() {
        return bestTarget;
    }

    /** 
     * Does not account for camera offset
     */
    public double getBestTargetX() {
        return bestTarget.bestCameraToTarget.getX();
    }
    /** 
     * Does not account for camera offset
     */
    public double getBestTargetY() {
        return bestTarget.bestCameraToTarget.getY();
    }

    public Transform3d getCamToTarget() {
        return bestTarget.getBestCameraToTarget();
    }

    // Returns list of IDs currently being tracked
    public List<Integer> getAprilTagIDs() {
        List<PhotonTrackedTarget> targets = getLatestResult().getTargets();
        List<Integer> tagIDs = new ArrayList<>();
        targets.forEach(target -> tagIDs.add(target.getFiducialId()));

        return tagIDs;
    }

    // Returns true if an the ID is being tracked
    public boolean containsID(Integer ID) {
        return getAprilTagIDs().contains(ID);
    }
}