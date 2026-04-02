// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Photonvision abstraction providing the best result and the camera offset for the result
 */
public class PhotonVision extends SubsystemBase {
    private static PhotonVision instance;

    private double hubDistance; // used by the shooter subsystem

    public static PhotonVision getInstance() {
        if(instance == null) {
            instance = new PhotonVision();
        }
        return instance;
    }

    public PhotonVision() {}

    @Override
    public void periodic() {}

    public double getHubDistance() {
        return hubDistance;
    }
}