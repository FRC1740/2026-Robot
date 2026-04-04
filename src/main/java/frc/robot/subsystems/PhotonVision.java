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
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

/**
 * Photonvision abstraction providing the best result and the camera offset for the result
 */
public class PhotonVision extends SubsystemBase {
    private static PhotonVision instance;

    private double hubDistance; // used by the shooter subsystem

    public boolean is_teleop;
  
    private ShuffleboardTab tab = Shuffleboard.getTab("Kicker");

    private final double[] m_poseArray = new double[3];

    private GenericEntry BLCamPose =
        tab.add("BL Cam Pose", m_poseArray)
            .getEntry();
    private GenericEntry FLCamPose =
    tab.add("FL Cam Pose", m_poseArray)
        .getEntry();
    private GenericEntry BRCamPose =
    tab.add("BR Cam Pose", m_poseArray)
        .getEntry();
    private GenericEntry FRCamPose =
    tab.add("FR Cam Pose", m_poseArray)
        .getEntry();
    
    // PhotonCamera backLeftCamera = new PhotonCamera("BackLeft");
    PhotonCamera frontLeftCamera = new PhotonCamera("FrontLeft");
    PhotonCamera backRightCamera = new PhotonCamera("BackRight");
    PhotonCamera frontRightCamera = new PhotonCamera("FrontRight");

    PhotonPoseEstimator backLeftCameraEstimator = 
        new PhotonPoseEstimator(VisionConstants.aprilTagFieldLayout, VisionConstants.RobotToBackLeftCamera);
    PhotonPoseEstimator frontLeftCameraEstimator = 
        new PhotonPoseEstimator(VisionConstants.aprilTagFieldLayout, VisionConstants.RobotToFrontLeftCamera);
    PhotonPoseEstimator backRightCameraEstimator = 
        new PhotonPoseEstimator(VisionConstants.aprilTagFieldLayout, VisionConstants.RobotToBackRightCamera);
    PhotonPoseEstimator frontRightCameraEstimator = 
        new PhotonPoseEstimator(VisionConstants.aprilTagFieldLayout, VisionConstants.RobotToFrontRightCamera);

    class Camera {
        PhotonCamera camera;
        
        PhotonPoseEstimator estimator;

        GenericEntry CamPose;

        Camera(PhotonCamera camera, PhotonPoseEstimator estimator, GenericEntry CamPose) {
            this.camera = camera;
            this.estimator = estimator;
            this.CamPose = CamPose;
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
        is_teleop = false;
        // cameras.add(new Camera(backLeftCamera, backLeftCameraEstimator, BLCamPose));
        cameras.add(new Camera(frontLeftCamera, frontLeftCameraEstimator, FLCamPose));
        cameras.add(new Camera(backRightCamera, backRightCameraEstimator, BRCamPose));
        cameras.add(new Camera(frontRightCamera, frontRightCameraEstimator, FRCamPose));
    }

    NetworkTable VisionTable = NetworkTableInstance.getDefault().getTable("Vision");

    StructArrayPublisher<Pose2d> FLPublisherPos = VisionTable
            .getStructArrayTopic("FLPublisherPos", Pose2d.struct).publish();
            
    StructArrayPublisher<Pose2d> FRPublisherPos = VisionTable
            .getStructArrayTopic("FRPublisherPos", Pose2d.struct).publish();

    @Override
    public void periodic() {

        FLPublisherPos.set(new Pose2d[] {
                new Pose2d(
                    VisionConstants.RobotToFrontLeftCamera.getX() + CommandSwerveDrivetrain.getInstance().getState().Pose.getX(),
                    VisionConstants.RobotToFrontLeftCamera.getY() + CommandSwerveDrivetrain.getInstance().getState().Pose.getY(), 
                    new Rotation2d(VisionConstants.RobotToFrontLeftCamera.getRotation().getMeasureZ()).rotateBy(CommandSwerveDrivetrain.getInstance().getState().Pose.getRotation()))
            });
        FRPublisherPos.set(new Pose2d[] {
                new Pose2d(
                    VisionConstants.RobotToFrontRightCamera.getX() + CommandSwerveDrivetrain.getInstance().getState().Pose.getX(),
                    VisionConstants.RobotToFrontRightCamera.getY() + CommandSwerveDrivetrain.getInstance().getState().Pose.getY(), 
                    new Rotation2d(VisionConstants.RobotToFrontRightCamera.getRotation().getMeasureZ()).rotateBy(CommandSwerveDrivetrain.getInstance().getState().Pose.getRotation()))
            });

        for (Camera camera : cameras) {
            List<PhotonPipelineResult> res = camera.camera.getAllUnreadResults();

            if (res.isEmpty()) {
                continue;
            }

            PhotonPipelineResult result = res.get(res.size() - 1);

            if (result != null) {
                PhotonTrackedTarget target = result.getBestTarget();
                if (target != null) {
                    if (target.area < 0.1) {
                        continue;
                    }
                }
            }

            Optional<EstimatedRobotPose> visionEst = camera.estimator.estimateCoprocMultiTagPose(result);
            
            if (visionEst.isEmpty()) {
                visionEst = camera.estimator.estimateLowestAmbiguityPose(result);
            }

            if (!visionEst.isEmpty()) {
                Pose2d pose = visionEst.get().estimatedPose.toPose2d();

                if (is_teleop) {
                    pose = new Pose2d(pose.getTranslation(), CommandSwerveDrivetrain.getInstance().getState().Pose.getRotation());
                }

                m_poseArray[0] = pose.getX();
                m_poseArray[1] = pose.getY();
                m_poseArray[2] = pose.getRotation().getDegrees();

                camera.CamPose.setDoubleArray(m_poseArray);
                if (backRightCamera.getName() != "BackRight") {
                    CommandSwerveDrivetrain.getInstance().addVisionMeasurement(pose, visionEst.get().timestampSeconds);
                }
            }
        }
    }

    public double getHubDistance() {
        Transform2d delta;
        if (CommandSwerveDrivetrain.getInstance().m_operatorPerspectiveFlipped) { // Red
            delta = CommandSwerveDrivetrain.getInstance().getState().Pose.minus(VisionConstants.RedHubPose);
        }else {
            delta = CommandSwerveDrivetrain.getInstance().getState().Pose.minus(VisionConstants.BlueHubPose);
        }
        
        hubDistance = Units.metersToInches(
            Math.sqrt(Math.pow(delta.getX(), 2) + Math.pow(delta.getY(), 2))
        ) - (47.0 / 2.0)// center of hub to the outer edge offset (0in is from edge)
          - (27.5 / 2) // robot has width
          - (12) // intake
        ; 

        return hubDistance;
    }
}