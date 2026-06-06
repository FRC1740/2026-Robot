// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kTestDriverControllerPort = 2;
    public static final int kCoDriverControllerPort = 1;
  }
  public static class CanIDs {
    public static int shooterRightMotor = 9;
    public static int shooterLeftMotor = 10;
    public static int feederMotor = 1;
    public static int kickerMotor = 11;
    public static int intakeExtensionMotor = 13;
    public static int intakeMotor = 14;
  }
  public static class Shooter {
    public static class CalibrationPoint {
      // distance in inches
      CalibrationPoint (double distance, double angle, double rpm) {
        this.distance = distance;
        this.angle = angle;
        
        this.rpm = rpm;
      }
      
      public double distance;
      public double angle;
      public double rpm;
    }
    // edge hub center flywheel
    // 30in, 0
    // 105in, .45

    // MUST BE IN ORDER CLOSEST TO FARTHEST
    public static final CalibrationPoint[] shooterCalibration = {
      new CalibrationPoint(0, 0.0, 4000.0),
      new CalibrationPoint(24, 0.0, 4600.0),
      new CalibrationPoint(48, 0.2, 4600.0),
      new CalibrationPoint(48+24, 0.3, 5400),
    };

    public static double lerp(double a, double b, double t) {
      return a + (b - a) * t;
    }


    public static CalibrationPoint getPoint(double distance) {
      distance = Math.max(distance, 0.00001);
      CalibrationPoint previousPoint = new CalibrationPoint(0.0, 0.0, 0.0);
      CalibrationPoint calibrationPoint = new CalibrationPoint(0.0, 0.0, 0.0);
      // loop over all points
      for (int i = 0; i < shooterCalibration.length; i++) {
        calibrationPoint = shooterCalibration[i];
        if (i == shooterCalibration.length - 1) {
          // ensure previousPoint != previousPoint so lerp extrapolates at the extremes
          break;
        }
        if (distance > calibrationPoint.distance) {
          previousPoint = calibrationPoint;
        }else {
         break; // we have reached the endpoint for the lerp
        }
      }

      // (Calibration point is larger), gets 0-1 position of the distance for interpolation
      double position = 
        (distance - previousPoint.distance) / 
        (calibrationPoint.distance - previousPoint.distance);

      return new CalibrationPoint(
        distance,
        lerp(previousPoint.angle, calibrationPoint.angle, position), // interpolate angle and rpm
        lerp(previousPoint.rpm, calibrationPoint.rpm, position)
      );
    }
  }

public static final class VisionConstants {
    // 10.17.40.2:5810
    // 10.17.40.11
    // http://photonvision.local:5800/#/dashboard
    public static final String camName = "Cam1"; // grey
    public static final String cam2Name = "Cam2"; // white
    public static final String cam3Name = "Cam3"; // left front
    public static final String cam4Name = "Cam4"; // right front
    public static final Double AprilTagMinimumArea = 0.0;

    public static AprilTagFieldLayout aprilTagFieldLayout = null;

    static {
        try {
            // if you set this, you may get incorrect tag positions!!!
            // make sure this is updated to the current game
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2026RebuiltAndymark.m_resourceFile);
        } catch (IOException IOE) {
            IOE.printStackTrace();
        }
    }

    public static final Pose2d BlueHubPose = new Pose2d(
      Units.inchesToMeters((325.61) - 143.5),
      Units.inchesToMeters((317.69 / 2.0)),
      new Rotation2d(0.0)
    );
    public static final Pose2d RedHubPose = new Pose2d(
      Units.inchesToMeters(((325.61) - 143.5) + 240.0 + 47.0), // 240 = dist between + 47 is width
      Units.inchesToMeters((317.69 / 2.0)),
      new Rotation2d(0.0)
    );

    public static final Transform3d RobotToFrontCamera = new Transform3d(Units.inchesToMeters(-1), 0.0, Units.inchesToMeters(30.0), new Rotation3d(0.0, Units.degreesToRadians(15.0), 0.0));
    
    public static final Transform3d RobotToBackRightCamera = new Transform3d(-.3, -.22, 19.5, new Rotation3d(0.0, 0.0, Math.PI));

    public static final Transform3d RobotToFrontLeftCamera = new Transform3d(0.0, 0.25, Units.inchesToMeters(7.5), new Rotation3d(0.0, .52, Math.PI / 2.0));
    
    public static final Transform3d RobotToFrontRightCamera = new Transform3d(0.0, -0.25, Units.inchesToMeters(7.5), new Rotation3d(0.0, .52, -Math.PI / 2.0));


    public static final Transform2d QuestToRobot = new Transform2d( /*TODO: Put x, y, rotational offsets here!*/ );

    public static final double questVisionUpdateThreshold = 0.1; // TODO! tune

    public enum AprilTagIDs {;

        private final int ID;

        AprilTagIDs(int ID) {
            this.ID = ID;
        }

        public int getID() {
            return ID;
        }
    }
}
}
