// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Telemetry;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  TalonFX rightMotor = new TalonFX(Constants.CanIDs.shooterRightMotor, "*"); 
  TalonFX leftMotor = new TalonFX(Constants.CanIDs.shooterLeftMotor, "*");

  boolean isToggled = false;

  private final Telemetry telemetry = Telemetry.getInstance();

  private final TorqueCurrentFOC m_torqueRequest = new TorqueCurrentFOC(0);

  private final HoodSubsystem m_hoodSubsystem = HoodSubsystem.getInstance();

  final VelocityVoltage VVShootRequest = new VelocityVoltage(0).withSlot(0);

  Slot1Configs slot1Configs = new Slot1Configs();
  Slot0Configs slot0Configs = new Slot0Configs();

  private ShuffleboardTab tab = Shuffleboard.getTab("Drive");

  private GenericEntry shooter_velocity =
      tab.add("Shooter Velocity", 0)
         .getEntry();
  private GenericEntry shooter_angle =
      tab.add("Shooter Angle", 0)
         .getEntry();
  
  private static ShooterSubsystem instance;

  public static ShooterSubsystem getInstance() {
    if(instance == null) {
      instance = new ShooterSubsystem();
    }
    return instance;
  }


  public ShooterSubsystem() {
    // set params here
    TalonFXConfigurator rightMotorconfigurator = rightMotor.getConfigurator();
    TalonFXConfigurator leftMotorconfigurator = leftMotor.getConfigurator();
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    // Incredibly important!!!!! 
    // Without this the motor draws as much power as it wants and will die if stalled
    motorConfig.CurrentLimits.StatorCurrentLimit = 50;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 1;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // PID
    
    slot0Configs.kP = 3.5; // An error of 1 rotation results in 2.4 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0; // A velocity of 1 rps results in 0.1 V output

    slot1Configs.kP = 0;
    slot1Configs.kI = 0;
    slot1Configs.kD = 0;


    // rightMotor.setControl(VVShootRequest.withVelocity(0));
    leftMotor.setControl(new Follower(Constants.CanIDs.shooterRightMotor, MotorAlignmentValue.Opposed));

    rightMotorconfigurator.apply(motorConfig);
    rightMotorconfigurator.apply(slot0Configs);
    leftMotorconfigurator.apply(motorConfig);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    telemetry.telemetrizeShooter(getCurrentVelocity());
  }

  /**
   * 
   * @return Velocity of the first encoder
   */

  public double getCurrentVelocity() {
    return rightMotor.getVelocity().getValueAsDouble() * 60.0; // RPS -> RPM
  }

  public void setTorque(double Torque) {
    rightMotor.setControl(m_torqueRequest.withOutput(Torque));
  }

  public void aimForDistance(double distance) {
    m_hoodSubsystem.setPercent(shooter_angle.getDouble(0));
  }

  public void shootDumb() {
    rightMotor.set(.2);
  }
  public void shoot() {
    rightMotor.setControl(VVShootRequest.withVelocity(-2000));
  }
  public void shootSB() {
    rightMotor.setControl(VVShootRequest.withVelocity(-shooter_velocity.getDouble(0) / 60.0));
  }

  public boolean atSpeed() {
    return (-rightMotor.getVelocity().getValueAsDouble() * 60.0) > (shooter_velocity.getDouble(0) / 60.0) - 200;
  }

  public boolean spinning() {
    return Math.abs(-rightMotor.getVelocity().getValueAsDouble() * 60.0) > 100;
  }

  public void toggle() {
    if (isToggled) {

      //Slot1configs are disabled
      rightMotor.getConfigurator().apply(slot1Configs);
      rightMotor.set(0);
    } else {

      //Slot0configs are enabled
      rightMotor.getConfigurator().apply(slot0Configs);
    }
    isToggled = !isToggled;
  }

  public void stop(){
    rightMotor.set(0);
  }
}
