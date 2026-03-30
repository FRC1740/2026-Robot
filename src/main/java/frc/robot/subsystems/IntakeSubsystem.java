// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Telemetry;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  
  SparkMax flipMotorController = new SparkMax(Constants.CanIDs.intakeExtensionMotor, MotorType.kBrushless); 
  SparkClosedLoopController flipMotorLoopController;
  TalonFX intakeMotorController = new TalonFX(Constants.CanIDs.intakeMotor,"*"); 
    private final TrapezoidProfile m_profile =
        new TrapezoidProfile(new TrapezoidProfile.Constraints(120.0, 30));
    private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

  private static IntakeSubsystem instance;
  private RelativeEncoder flipMotorEncoder;

  private boolean isFlippedDown = false;

  Slot0Configs slot0Configs = new Slot0Configs();

  Timer intakeRollersStallTimer = new Timer();

  boolean ejecting = false;

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
    intakeMotorConfig.CurrentLimits.StatorCurrentLimit = 70;
    intakeMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    intakeMotorConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = .25;
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
    flipMotorConfig.softLimit.reverseSoftLimit(-47);
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
       flipMotorController.getOutputCurrent(),
       intakeMotorController.getVelocity().getValueAsDouble(), ejecting, intakeRollersStallTimer.get());

    // output > 50A
    if (!ejecting) {
      if (intakeMotorController.getStatorCurrent().getValueAsDouble() > 50.0) {
        intakeRollersStallTimer.start();
      }else {
        intakeRollersStallTimer.reset();
        intakeRollersStallTimer.stop();
      }

      // stalled for .3s, so eject
      if (intakeRollersStallTimer.hasElapsed(0.7)) {
        ejecting = true;
        intakeRollersStallTimer.reset();
        intakeRollersStallTimer.start();
      }
    }else { // ejecting == true
      // eject for 1s
      if (intakeRollersStallTimer.hasElapsed(.5)) {
        ejecting = false;
        intakeRollersStallTimer.reset();
        intakeRollersStallTimer.stop();
      }
    }
  }

  public void seekPosition() {
    m_setpoint = m_profile.calculate(0.02, m_setpoint, m_goal);
    flipMotorLoopController.setSetpoint(m_setpoint.position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public double getCurrentVelocity() {
    return intakeMotorController.getVelocity().getValueAsDouble() * 60.0; // RPS -> RPM
  }

  public void spinIntake() {
    // override if ejecting
    if (ejecting) {
      spit();
      return;
    }
    intakeMotorController.setControl(new DutyCycleOut(1).withEnableFOC(true));
  }

  public void spit() {
      intakeMotorController.setControl(new DutyCycleOut(-1).withEnableFOC(true));
  }

  public void stopIntake() {
    intakeMotorController.setControl(new DutyCycleOut(0).withEnableFOC(true));
  }

  public void stopFlip() {
    flipMotorLoopController.setSetpoint(-13 * 3, ControlType.kPosition, ClosedLoopSlot.kSlot2);
  }

  public void flipDown() {
    m_goal = new TrapezoidProfile.State(-15 * 3, 0);
  }

  public void flipUp() {
    m_goal = new TrapezoidProfile.State(0, 0);
  }
  
  public void latch() {
    // flipMotorLoopController.setSetpoint(0.1, ControlType.kCurrent, ClosedLoopSlot.kSlot1);
  }
}
