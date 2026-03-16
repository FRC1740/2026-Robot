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
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Telemetry;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  
  SparkMax flipMotorController = new SparkMax(Constants.CanIDs.intakeExtensionMotor, MotorType.kBrushless); 
  SparkClosedLoopController flipMotorLoopController;
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
    flipMotorConfig.smartCurrentLimit(50);
    flipMotorConfig.softLimit.forwardSoftLimitEnabled(true);
    flipMotorConfig.softLimit.reverseSoftLimitEnabled(true);
    flipMotorConfig.encoder.positionConversionFactor(3);
    flipMotorConfig.openLoopRampRate(0.5);
    flipMotorConfig.softLimit.forwardSoftLimit(0);
    flipMotorConfig.softLimit.reverseSoftLimit(-15 * 3);
    flipMotorConfig.idleMode(IdleMode.kBrake);

    flipMotorConfig.closedLoop
      
      .p(0.02, ClosedLoopSlot.kSlot0)
      .i(0.0, ClosedLoopSlot.kSlot0)
      .d(0.0, ClosedLoopSlot.kSlot0)
      // intake in
      .p(0.04, ClosedLoopSlot.kSlot1)
      .i(0.0, ClosedLoopSlot.kSlot1)
      .d(0.0, ClosedLoopSlot.kSlot1)

      .p(0.0, ClosedLoopSlot.kSlot2)
      .i(0.0, ClosedLoopSlot.kSlot2)
      .d(0.0, ClosedLoopSlot.kSlot2);
    
    flipMotorController.configure(flipMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    flipMotorLoopController = flipMotorController.getClosedLoopController();
    

    flipMotorEncoder = flipMotorController.getEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    telemetry.telemetrizeIntake(flipMotorEncoder.getPosition(), 
      intakeMotorController.getStatorCurrent().getValueAsDouble(),
       flipMotorController.getOutputCurrent());
  }

  public double getCurrentVelocity() {
    return intakeMotorController.getVelocity().getValueAsDouble() * 60.0; // RPS -> RPM
  }

  public void spinIntake() {
    // if (flipMotorLoopController.isAtSetpoint() || 
    //     // Slot 1 is latch so it's auto good
    //     flipMotorLoopController.getSelectedSlot() == ClosedLoopSlot.kSlot1) {

      intakeMotorController.set(1);
    // }
  }

  public void spit() {
      intakeMotorController.set(-1);
  }

  public void stopIntake() {
    intakeMotorController.set(0);
  }

  public void stopFlip() {
    flipMotorLoopController.setSetpoint(-13 * 3, ControlType.kPosition, ClosedLoopSlot.kSlot2);
  }

  public void flipDown() {
    flipMotorLoopController.setSetpoint(-15 * 3, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public void flipUp() {
    flipMotorLoopController.setSetpoint(0, ControlType.kPosition, ClosedLoopSlot.kSlot1);
  }
  
  public void latch() {
    // flipMotorLoopController.setSetpoint(0.1, ControlType.kCurrent, ClosedLoopSlot.kSlot1);
  }
}
