// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Telemetry;
import frc.robot.Constants;

public class FeederSubsystem extends SubsystemBase {
  
  // Use id 0 to adress the motor via canbus, this is a neo so brushless.
  // Ideally we configure via a constants file but this is example code.
  SparkMax motorController = new SparkMax(Constants.CanIDs.feederMotor, MotorType.kBrushless); 
  private final Telemetry telemetry = Telemetry.getInstance();

  private RelativeEncoder motorEncoder;

  private ShuffleboardTab tab = Shuffleboard.getTab("Feeder");

  private GenericEntry feederSpeed =
      tab.add("Feeder Speed", 1)
         .getEntry();

  private static FeederSubsystem instance;

  public static FeederSubsystem getInstance() {
    if(instance == null) {
      instance = new FeederSubsystem();
    }
    return instance;
  }


  public FeederSubsystem() {
    // set params here
    SparkMaxConfig config = new SparkMaxConfig();

    // Incredibly important!!!!! 
    // Without this the motor draws as much power as it wants and will die if stalled
    config.smartCurrentLimit(40);
    config.openLoopRampRate(.1);
    
    motorController.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    motorEncoder = motorController.getEncoder();
    
  }

  public double getCurrentVelocity() {
    return motorEncoder.getVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // telemetry.telemetrizeIntake(getCurrentVelocity());
  }

  public void feed() {
    motorController.set(-feederSpeed.getDouble(0));
  }
  public void spit() {
    motorController.set(0.4);
  }


  public void stop() {
    motorController.set(0); // motorController.stopMotor(); also works
  }
}
