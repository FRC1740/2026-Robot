// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.Telemetry;

public class KickerSubsystem extends SubsystemBase {
  // Creates a new TalonFX object
  TalonFXS kickerMotorController = new TalonFXS(Constants.CanIDs.feederMotor, "*");
  
  //Create control request
  DutyCycleOut intakMotorDutyCyleOut = new DutyCycleOut(0.0);
  
  private final Telemetry telemetry = Telemetry.getInstance();
  private static Double motorVelocity;
  private static KickerSubsystem instance;

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

    kickerMotorController.getConfigurator().apply(kickerMotorConfig);

    motorVelocity = kickerMotorController.getVelocity().getValueAsDouble();
  }

  @Override
  public void periodic() {
    telemetry.telemetrizeKicker(getCurrentVelocity());
  }

  public double getCurrentVelocity() {
    return motorVelocity / 60.0;
  }

  public void kick() {
    // Starts the Motor
    kickerMotorController.setControl(intakMotorDutyCyleOut.withOutput(1.0));

  }

  public void stop() {
    kickerMotorController.stopMotor();
  }
}