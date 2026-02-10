// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.Telemetry;

public class IntakeSubsystem extends SubsystemBase {
  // Creates a new TalonFX object
  TalonFXS intakeMotorController = new TalonFXS(Constants.OperatorConstants.intakeMotorID);
  
  //Create control request
  DutyCycleOut intakMotorDutyCyleOut = new DutyCycleOut(0.0);
  
  private final Telemetry telemetry = Telemetry.getInstance();
  private static Double motorVelocity;
  private static IntakeSubsystem instance;

  public static IntakeSubsystem getInstance() {
    if(instance == null) {
      instance = new IntakeSubsystem();
    }
    return instance;
  }

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    // Make new TalonFX config objects
    TalonFXSConfiguration intakeMotorConfig = new TalonFXSConfiguration();

    // Set the current limit of the Talon
    intakeMotorConfig.CurrentLimits.SupplyCurrentLimit = 20;
    intakeMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    intakeMotorController.getConfigurator().apply(intakeMotorConfig);

    motorVelocity = intakeMotorController.getVelocity().getValueAsDouble();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Gather RPM dataof the intake motor
    telemetry.telemetrizeIntake(getCurrentVelocity());
  }

  public double getCurrentVelocity() {
    // motorVelocity multiplied by 60 to get RPM instead of RPS

    return motorVelocity * 60.0;
  }

  public void intake() {
    // Starts the Motor

    //intakeMotorController.set(-1);

    intakeMotorController.setControl(intakMotorDutyCyleOut.withOutput(.5));

  }

  public void stop() {
    // Stops the Motor

    intakeMotorController.stopMotor();
  }
}
