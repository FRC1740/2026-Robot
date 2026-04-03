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
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

/**
 * Photonvision abstraction providing the best result and the camera offset for the result
 */
public class PhotonVision extends SubsystemBase {
    private static PhotonVision instance;

    private double hubDistance; // used by the shooter subsystem

    // PhotonCamera backLeftCamera = new PhotonCamera("BackLeft");
    PhotonCamera frontLeftCamera = new PhotonCamera("FrontLeft");
    PhotonCamera backRightCamera = new PhotonCamera("BackRight");
    PhotonCamera frontRightCamera = new PhotonCamera("FrontRight");

    // PhotonPoseEstimator backLeftCameraEstimator = 
    //     new PhotonPoseEstimator(VisionConstants.aprilTagFieldLayout, VisionConstants.RobotToBackLeftCamera);
    PhotonPoseEstimator frontLeftCameraEstimator = 
        new PhotonPoseEstimator(VisionConstants.aprilTagFieldLayout, VisionConstants.RobotToFrontLeftCamera);
    PhotonPoseEstimator backRightCameraEstimator = 
        new PhotonPoseEstimator(VisionConstants.aprilTagFieldLayout, VisionConstants.RobotToBackRightCamera);
    PhotonPoseEstimator frontRightCameraEstimator = 
        new PhotonPoseEstimator(VisionConstants.aprilTagFieldLayout, VisionConstants.RobotToFrontRightCamera);

    class Camera {
        PhotonCamera camera;
        
        PhotonPoseEstimator estimator;

        Camera(PhotonCamera camera, PhotonPoseEstimator estimator) {
            this.camera = camera;
            this.estimator = estimator;
        }
    }

    List<Camera> cameras = new ArrayList<>();

    public static PhotonVision getInstance() {
        if(instance == null) {
            instance = new PhotonVision();
        }
        return instance;
    }

    public PhotonVision() {
        // cameras.add(new Camera(backLeftCamera, backLeftCameraEstimator));
        cameras.add(new Camera(frontLeftCamera, frontLeftCameraEstimator));
        cameras.add(new Camera(backRightCamera, backRightCameraEstimator));
        cameras.add(new Camera(frontRightCamera, frontRightCameraEstimator));
    }

    @Override
    public void periodic() {
        for (Camera camera : cameras) {
            List<PhotonPipelineResult> res = camera.camera.getAllUnreadResults();

            if (res.isEmpty()) {
                continue;
            }

            PhotonPipelineResult result = res.get(res.size() - 1);
            
            Optional<EstimatedRobotPose> visionEst = camera.estimator.estimateCoprocMultiTagPose(result);
            if (visionEst.isEmpty()) {
                visionEst = camera.estimator.estimateLowestAmbiguityPose(result);
            }

            if (!visionEst.isEmpty()) {
                CommandSwerveDrivetrain.getInstance().addVisionMeasurement(visionEst.get().estimatedPose.toPose2d(), visionEst.get().timestampSeconds);
            }
        }
    }

    public double getHubDistance() {
        return hubDistance;
    }
}