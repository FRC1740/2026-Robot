// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class KickerSubsystem extends SubsystemBase {
  /** Creates a new kickerSubsystem. */

  TalonFX kickerMotor = new TalonFX(11, "*");
  private static KickerSubsystem instance;

  private DutyCycleOut dutycycle = new DutyCycleOut(0);


  public static KickerSubsystem getInstance() {
    if(instance == null) {
      instance = new KickerSubsystem();
    }
    return instance;
  }

  public KickerSubsystem() {
    TalonFXConfigurator kickerMotorConfigurator = kickerMotor.getConfigurator();
    TalonFXConfiguration kickerMotorConfig = new TalonFXConfiguration();
    // Incredibly important!!!!! 
    // Without this the motor draws as much power as it wants and will die if stalled
    kickerMotorConfig.CurrentLimits.StatorCurrentLimit = 20;
    kickerMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    kickerMotorConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 1;
    kickerMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;


    kickerMotor.setControl(dutycycle);

    kickerMotorConfigurator.apply(kickerMotorConfig);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void feed() {
    kickerMotor.setControl(dutycycle.withOutput(1.0));
  }

  public void stop() {
    kickerMotor.set(0);
  }
}
