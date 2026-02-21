// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;

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
    static class CalibrationPoint {
      CalibrationPoint (double distance, double angle, double rpm) {
        this.distance = distance;
        this.angle = angle;
        this.rpm = rpm;
      }
      
      double distance;
      double angle;
      double rpm;
    }
    // edge hub center flywheel
    // 15.3ft, .5

    public static final CalibrationPoint[] shooterCalibration = {
      new CalibrationPoint(1.0, 0.0, 2000.0),
    };

    // public static CalibrationPoint getPoint(double distance) {
      
    // }
  }

public static final class VisionConstants {
    // 10.17.40.2:5810
    // 10.17.40.11
    // http://photonvision.local:5800/#/dashboard
    public static final String camName = "Cam1"; // grey
    public static final String cam2Name = "Cam2"; // white
    public static final Double AprilTagMinimumArea = 0.0;

    public static final Double cam12Dist = 0.22; // dist from cam1 to cam2 in meters
    public static final Double cam12FrontBackOffset = 0.2; // dist of the two cameras from middle of the robot

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

    public static final Transform3d RobotToCam1 = new Transform3d(cam12FrontBackOffset, -cam12Dist, 0.0, new Rotation3d(0.0, 0.0, -.0));
    
    public static final Transform3d RobotToCam2 = new Transform3d(cam12FrontBackOffset, cam12Dist, 0.0, new Rotation3d(0.0, 0.0, .0));

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
