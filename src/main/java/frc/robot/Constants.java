// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

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

    public static final int intakeMotorID = 1;

  }
  public static class CanIDs {
    public static int shooterRightMotor = 9;
    public static int shooterLeftMotor = 10;
    public static int feederMotor = 11;
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

    // 10ft .3 3000
    // 0.3 ft 0 2300

    public static final CalibrationPoint[] shooterCalibration = {
      new CalibrationPoint(1.0, 0.0, 2000.0),
    };

    // public static CalibrationPoint getPoint(double distance) {
      
    // }
  }
}
