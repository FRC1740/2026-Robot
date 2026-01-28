// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.IntakeSubsystem;

/** Add your docs here. */
public class Telemetry {
    private static Telemetry instance;
    NetworkTableInstance ins = NetworkTableInstance.getDefault();
    NetworkTable intakeTable = ins.getTable("Intake table");


    DoublePublisher intakeRPM = intakeTable.getDoubleTopic("intake rpm").publish();
    
    public static Telemetry getInstance() {
        if(instance == null) {
         instance = new Telemetry();
        }
    return instance;
    }

    public void telemetrizeIntake(double rpm) {
        intakeRPM.set(rpm);
    }
}