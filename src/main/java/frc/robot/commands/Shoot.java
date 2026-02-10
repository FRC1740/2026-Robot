// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Shoot extends Command {
  ShooterSubsystem m_shooterSubsystem;
  KickerSubsystem m_kickerSubsystem;

  /** Creates a new Shoot. */
  public Shoot(ShooterSubsystem shooterSubsystem, KickerSubsystem kickerSubsystem) {
    m_shooterSubsystem = shooterSubsystem;
    m_kickerSubsystem = kickerSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    // Prevents double accesses
    addRequirements(m_shooterSubsystem, m_kickerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooterSubsystem.shootSB();
    m_kickerSubsystem.feed();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.stop();
    m_kickerSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
