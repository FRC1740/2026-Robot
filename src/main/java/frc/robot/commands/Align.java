package frc.robot.commands;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Align extends Command {

  CommandSwerveDrivetrain m_drivetrain;
  SwerveRequest.FieldCentric m_drive;
  CommandXboxController m_driverController;
  Transform2d target;

  /** Creates a new Shoot. */
  public Align(CommandSwerveDrivetrain drivetrain, SwerveRequest.FieldCentric drive, CommandXboxController driverController) {

    m_drivetrain = drivetrain;
    m_drive = drive;
    m_driverController = driverController;
    

    // Use addRequirements() here to declare subsystem dependencies.
    // Prevents double accesses
    // addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    target = new Pose2d(10.0, 10.0, new Rotation2d(0.0)) // hub
                                    .minus(m_drivetrain.getState().Pose);

    double delta = Math.atan2(target.getY(), target.getX()) - m_drivetrain.getState().Pose.getRotation().getRadians();

    m_drivetrain.setControl(
      m_drive
        .withVelocityX(-m_driverController.getLeftY() * RobotContainer.MaxSpeed) // Drive forward with negative Y (forward)
        .withVelocityY(-m_driverController.getLeftX() * RobotContainer.MaxSpeed)
        .withRotationalRate(delta * 10));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}