// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.Telemetry;

public class KickerSubsystem extends SubsystemBase {
  // Creates a new TalonFX object
  TalonFXS kickerMotorController = new TalonFXS(Constants.CanIDs.kickerMotor, "*");
  
  //
  Slot0Configs slot0Configs = new Slot0Configs();

  final VelocityTorqueCurrentFOC VVKickerRequest = new VelocityTorqueCurrentFOC(0).withSlot(0);
  
  private final Telemetry telemetry = Telemetry.getInstance();
  private static Double motorVelocity;
  private static KickerSubsystem instance;

  private ShuffleboardTab tab = Shuffleboard.getTab("Kicker");

  private GenericEntry kicker_velocity =
      tab.add("Kicker Speed", 1)
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
    TalonFXSConfiguration kickerMotorConfig = new TalonFXSConfiguration();

    // Set the current limit of the Talon
    kickerMotorConfig.CurrentLimits.SupplyCurrentLimit = 40;
    kickerMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    kickerMotorConfig.Commutation.MotorArrangement = MotorArrangementValue.NEO_JST;

    //PID
    slot0Configs.kP = 3.5; // An error of 1 rotation results in 2.4 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0; // A velocity of 1 rps results in 0.1 V output

    kickerMotorController.getConfigurator().apply(kickerMotorConfig);

    motorVelocity = kickerMotorController.getVelocity().getValueAsDouble();
  }

  @Override
  public void periodic() {
    telemetry.telemetrizeKicker(getCurrentVelocity(), kickerMotorController.getStatorCurrent().getValueAsDouble());
  }

  public double getCurrentVelocity() {
    return motorVelocity / 60.0;
  }

  public void kick() {
    // Starts the Motor
    kickerMotorController.setControl(VVKickerRequest.withVelocity(-kicker_velocity.getDouble(0) / 60.0));

  }

  public void stop() {
    kickerMotorController.stopMotor();
  }
}