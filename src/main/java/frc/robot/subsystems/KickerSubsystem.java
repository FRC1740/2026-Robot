// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.Telemetry;

public class KickerSubsystem extends SubsystemBase {
  // Creates a new TalonFX object
  TalonFX kickerMotorController = new TalonFX(Constants.CanIDs.kickerMotor, "*");
  
  //
  Slot0Configs slot0Configs = new Slot0Configs();

  final VelocityTorqueCurrentFOC VVKickerRequest = new VelocityTorqueCurrentFOC(0).withSlot(0);
  
  private final Telemetry telemetry = Telemetry.getInstance();
  private static KickerSubsystem instance;

  private ShuffleboardTab tab = Shuffleboard.getTab("Kicker");

  private GenericEntry kickerVelocity =
      tab.add("Kicker Speed", 2500)
         .getEntry();

  public static KickerSubsystem getInstance() {
    if(instance == null) {
      instance = new KickerSubsystem();
    }
    return instance;
  }

  public KickerSubsystem() {
    // Make new TalonFX config objects

    // NEEDS CUSTOM MOTOR WITH NEO IN THE PHENOIX
    TalonFXConfiguration kickerMotorConfig = new TalonFXConfiguration();

    // Set the current limit of the Talon
    kickerMotorConfig.CurrentLimits.SupplyCurrentLimit = 60;
    kickerMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    //PID
    slot0Configs.kP = 3.5; // An error of 1 rotation results in 2.4 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0; // A velocity of 1 rps results in 0.1 V output
    slot0Configs.kV = 0.12; 

    kickerMotorController.getConfigurator().apply(kickerMotorConfig);
    kickerMotorController.getConfigurator().apply(slot0Configs);
    
  }

  @Override
  public void periodic() {
    telemetry.telemetrizeKicker(getCurrentVelocity(), kickerMotorController.getStatorCurrent().getValueAsDouble());
  }

  public double getCurrentVelocity() {
    return kickerMotorController.getVelocity().getValueAsDouble() / 60.0;
  }

  public void kick() {
    // Starts the Motor
    kickerMotorController.setControl(VVKickerRequest.withVelocity(-kickerVelocity.getDouble(2500) / 60.0));

  }


  public void spit() {
    // Starts the Motor
    kickerMotorController.setControl(VVKickerRequest.withVelocity(700));

  }

  public void stop() {
    kickerMotorController.stopMotor();
  }
}