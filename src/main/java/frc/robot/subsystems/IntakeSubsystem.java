// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Telemetry;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  
  SparkMax extensionMotorController = new SparkMax(Constants.CanIDs.intakeExtensionMotor, MotorType.kBrushless); 
  SparkMax motorController = new SparkMax(Constants.CanIDs.intakeMotor, MotorType.kBrushless); 
  private static IntakeSubsystem instance;
  private RelativeEncoder motorEncoder;

  private final Telemetry telemetry = Telemetry.getInstance();
  /** Creates a new FeederSubsystem. */

  public static IntakeSubsystem getInstance() {
    if(instance == null) {
      instance = new IntakeSubsystem();
    }
    return instance;
  }

  public IntakeSubsystem() {
    // set params here
    SparkMaxConfig config = new SparkMaxConfig();

    // Incredibly important!!!!! 
    // Without this the motor draws as much power as it wants and will die if stalled
    config.smartCurrentLimit(20);
    config.softLimit.forwardSoftLimitEnabled(true);
    config.softLimit.reverseSoftLimitEnabled(true);
    config.encoder.positionConversionFactor(3);
    config.softLimit.forwardSoftLimit(0);
    config.softLimit.reverseSoftLimit(-100);
    config.idleMode(IdleMode.kBrake);
    
    extensionMotorController.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    motorEncoder = extensionMotorController.getEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    telemetry.telemetrizeIntake(motorEncoder.getPosition());
  }

  public void intake() {
    extensionMotorController.set(-.4);
    motorController.set(.8);
  }

  public void retract() {
    extensionMotorController.set(.4);
    motorController.set(0);
  }
  public void stop() {
    extensionMotorController.set(0);
    motorController.set(0);
  }
}
