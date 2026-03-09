// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
  
  SparkMax flipMotorController = new SparkMax(Constants.CanIDs.intakeExtensionMotor, MotorType.kBrushless); 
  TalonFX intakeMotorController = new TalonFX(Constants.CanIDs.intakeMotor,"*"); 

  private static IntakeSubsystem instance;
  private RelativeEncoder flipMotorEncoder;

  private boolean isFlippedDown = false;

  Slot0Configs slot0Configs = new Slot0Configs();

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
    SparkMaxConfig flipMotorConfig = new SparkMaxConfig();

    TalonFXConfigurator intakeMotorConfigurator = intakeMotorController.getConfigurator();
    TalonFXConfiguration intakeMotorConfig = new TalonFXConfiguration();

    //Talon Config
    intakeMotorConfig.CurrentLimits.StatorCurrentLimit = 50;
    intakeMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    intakeMotorConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 1;
    intakeMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    intakeMotorConfigurator.apply(intakeMotorConfig);
    intakeMotorConfigurator.apply(slot0Configs);
    
    // Incredibly important!!!!! 
    // Without this the motor draws as much power as it wants and will die if stalled

    //SparksMax Config
    flipMotorConfig.smartCurrentLimit(20);
    flipMotorConfig.softLimit.forwardSoftLimitEnabled(true);
    flipMotorConfig.softLimit.reverseSoftLimitEnabled(true);
    flipMotorConfig.encoder.positionConversionFactor(3);
    flipMotorConfig.softLimit.forwardSoftLimit(0);
    flipMotorConfig.softLimit.reverseSoftLimit(-130.33334);
    flipMotorConfig.idleMode(IdleMode.kBrake);
    
    flipMotorController.configure(flipMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);



    flipMotorEncoder = flipMotorController.getEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    telemetry.telemetrizeIntake(getCurrentVelocity(), 
      intakeMotorController.getStatorCurrent().getValueAsDouble(),
       flipMotorController.getOutputCurrent());
  }

  public double getCurrentVelocity() {
    return intakeMotorController.getVelocity().getValueAsDouble() * 60.0; // RPS -> RPM
  }

  public void spinIntake() {
    if (isFlippedDown) {
      intakeMotorController.set(.6);
    }
  }

  public void stopIntake() {
    intakeMotorController.set(0);
  }

  public void toggleFlip() {
    //lwky don't know
    double currentPosition = flipMotorEncoder.getPosition();

    if (isFlippedDown) {

      if (currentPosition > targetPos) {
        flipMotorController.set(.1);
      } else {
        flipMotorController.set(0);
      }

    } else {

      if (currentPosition < targetPos) {
        flipMotorController.set(-.1);
      } else {
        flipMotorController.set(0);
      }

    }

    isFlippedDown = !isFlippedDown;
  }
}
