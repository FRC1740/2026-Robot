// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HoodSubsystem extends SubsystemBase {
  /** Creates a new FeederSubsystem. */

  private static HoodSubsystem instance;

  Servo servo = new Servo(0);
  Servo servo2 = new Servo(2);
  final double far_distance = 0; // out
  final double close_distance = 1; // in

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
    servo.set(far_distance);
    servo2.set(close_distance - .05);
  }

  public void setClose() {  // inwards 
    servo.set(close_distance);
    servo2.set(far_distance - .05);
  }

  private void set(double angle, double angle2) {
    servo.set(angle);
    servo2.set((angle2 - .1));
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
    percent = Math.min(Math.max(percent, 0.0), 1.0);
    set(
      ((close_distance - far_distance) * percent) + far_distance,
      ((close_distance - far_distance) * (1.0 - percent)) + far_distance
    );
  }
}
