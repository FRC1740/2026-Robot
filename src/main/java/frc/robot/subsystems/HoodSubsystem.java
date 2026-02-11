// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HoodSubsystem extends SubsystemBase {
  /** Creates a new FeederSubsystem. */

  private static HoodSubsystem instance;

  Servo servo = new Servo(0);
  Servo servo2 = new Servo(1);
  final double far_distance = .465; // out
  final double close_distance = .55; // in

  public static HoodSubsystem getInstance() {
    if(instance == null) {
      instance = new HoodSubsystem();
    }
    return instance;
  }

  public HoodSubsystem() {
    // servo.setBoundsMicroseconds(2400, 0, 0, 0, 700);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public void run() {}

  public void setFar() {
    servo.set(.465);
    servo2.set(.55);
  }

  public void setClose() {  // inwards 
    servo.set(.55);
    servo2.set(.465);
  }

  private void set(double angle, double angle2) {
    servo.set(angle);
    servo2.set(angle2);
  }

  private void setAngle(double degrees, Servo the_servo) {
    if (degrees < 0) {
      degrees = 0;
    } else if (degrees > 360*6) {
      degrees = 360*6;
    }

    the_servo.setPosition(degrees / 360*6);
  }

  /**
   * Takes in a 0-1 float representing the hood angle from 0 (least angled), to 1, most angled and flat
   */
  public void setPercent(double percent) {
    set(
      ((close_distance - far_distance) * percent) + far_distance,
      ((close_distance - far_distance) * (1.0 - percent)) + far_distance
    );
  }
}
