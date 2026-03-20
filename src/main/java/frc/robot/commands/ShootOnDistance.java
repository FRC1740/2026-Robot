// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootOnDistance extends Command {
  ShooterSubsystem m_shooterSubsystem;
  KickerSubsystem m_kickerSubsystem;
  FeederSubsystem m_feederSubsystem;

  Pose2d Hub;

  /** Creates a new Shoot. */
  public ShootOnDistance(ShooterSubsystem shooterSubsystem, KickerSubsystem kickerSubsystem, FeederSubsystem feederSubsystem) {
    m_shooterSubsystem = shooterSubsystem;
    m_kickerSubsystem = kickerSubsystem;
    m_feederSubsystem = feederSubsystem;
    

    // Use addRequirements() here to declare subsystem dependencies.
    // Prevents double accesses
    addRequirements(m_shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (CommandSwerveDrivetrain.getInstance().m_operatorPerspectiveFlipped) { // Red
      Hub = Constants.VisionConstants.RedHubPose;
    }else {
      Hub = Constants.VisionConstants.BlueHubPose;
    }
    Transform2d delta = CommandSwerveDrivetrain.getInstance().getState().Pose.minus(Hub);
    m_shooterSubsystem.shootByCalibration(
      Math.sqrt(Math.pow(delta.getX(), 2) + Math.pow(delta.getY(), 2))
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // m_shooterSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
