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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Telemetry;

public class IntakeSubsystem extends SubsystemBase {

  SparkMax intakeMotorController = new SparkMax(Constants.OperatorConstants.intakeMotorID, MotorType.kBrushless);
  private final Telemetry telemetry = Telemetry.getInstance();

  private static IntakeSubsystem instance;
  private RelativeEncoder motorEncoder;

  public static IntakeSubsystem getInstance() {
    if(instance == null) {
      instance = new IntakeSubsystem();
    }
    return instance;
  }

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();

    intakeMotorConfig.smartCurrentLimit(20);
    
    intakeMotorController.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    motorEncoder = intakeMotorController.getEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    telemetry.telemetrizeIntake(getCurrentVelocity());
  }

  public double getCurrentVelocity() {
    return motorEncoder.getVelocity();
  }

  public void intake() {
    intakeMotorController.set(-1);
  }

  public void stop() {
    intakeMotorController.set(0); // motorController.stopMotor(); also works
  }
}
