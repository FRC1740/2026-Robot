// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Add your docs here. */
public class Telemetry {
    private static Telemetry instance;
    NetworkTableInstance ins = NetworkTableInstance.getDefault();
    NetworkTable shooterTable = ins.getTable("Shooter table");
    NetworkTable intakeTable = ins.getTable("Intake table");

    DoublePublisher flywheelRPM = shooterTable.getDoubleTopic("Flywheel rpm").publish();
    DoublePublisher intakeRPM = intakeTable.getDoubleTopic("intake rpm").publish();
    
    public static Telemetry getInstance() {
    if(instance == null) {
        instance = new Telemetry();
    }
    return instance;
  }

    public void telemetrizeShooter(double rpm) {
        flywheelRPM.set(rpm);
    }

    public void telemetrizeIntake(double rpm) {
        intakeRPM.set(rpm);
    }
}
