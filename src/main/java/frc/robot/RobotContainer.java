// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.Telemetry;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Align;
import frc.robot.commands.Feed;
import frc.robot.commands.Intake;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShootOn;
import frc.robot.commands.TestShoot;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.ShooterSubsystem;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.opencv.core.Point;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.fasterxml.jackson.annotation.ObjectIdGenerators.None;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.configs.ParentConfiguration;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final ShooterSubsystem m_shooterSubsystem = ShooterSubsystem.getInstance();
    private final KickerSubsystem m_kickerSubsystem = KickerSubsystem.getInstance();
    private final FeederSubsystem m_feederSubsystem = FeederSubsystem.getInstance();
    private final IntakeSubsystem m_intakeSubsystem = IntakeSubsystem.getInstance();
    private final Telemetry m_telemetry = Telemetry.getInstance();

    public final PhotonVision photonvision = PhotonVision.getInstance();

    double time = 0.0;

    public static double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = 1.0 * RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    
    private final SwerveRequest.FieldCentricFacingAngle align = new SwerveRequest.FieldCentricFacingAngle()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_coDriverController =
      new CommandXboxController(OperatorConstants.kCoDriverControllerPort);
  private final CommandXboxController m_testController =
      new CommandXboxController(OperatorConstants.kTestDriverControllerPort);

    //   /* Path follower */ 
    SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    drivetrain.configureAutoBuilder();

    NamedCommands.registerCommand("Shoot", new Shoot(m_shooterSubsystem, m_kickerSubsystem, m_feederSubsystem));
    NamedCommands.registerCommand("Feed", new Feed(m_shooterSubsystem, m_kickerSubsystem, m_feederSubsystem));
    NamedCommands.registerCommand("Intake", new Intake(m_intakeSubsystem));

    // Configure the trigger bindings
    autoChooser = AutoBuilder.buildAutoChooser("Tests");
    
    SmartDashboard.putData("Auto Mode", autoChooser);

    configureBindings();
}

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    //Shooter buttons
    m_coDriverController.leftTrigger().whileTrue(new ShootOn(m_shooterSubsystem, m_kickerSubsystem, m_feederSubsystem));
    m_coDriverController.rightTrigger().onTrue(new InstantCommand(() -> {m_shooterSubsystem.toggle();}));
    m_coDriverController.povUp().onTrue(new InstantCommand(() -> {m_shooterSubsystem.increaseSpeed();}));
    m_coDriverController.povDown().onTrue(new InstantCommand(() -> {m_shooterSubsystem.decreaseSpeed();}));
    m_coDriverController.povRight().onTrue(new InstantCommand(() -> {m_shooterSubsystem.increaseAngle();}));
    m_coDriverController.povLeft().onTrue(new InstantCommand(() -> {m_shooterSubsystem.decreaseAngle();}));

    //Left trigger activates the flywheel of the shooter
    m_testController.leftTrigger().whileTrue(new TestShoot(m_shooterSubsystem));

    //A button toggles it on/off
    m_testController.a().onTrue(new InstantCommand(() -> {m_shooterSubsystem.toggle();}));

    //X button increases the speed by 100 RPM
    m_testController.x().onTrue(new InstantCommand(() -> {m_shooterSubsystem.increaseSpeed();}));
    

    // m_driverController.a().whileTrue(
    //     new Align(drivetrain, drive, m_driverController)
    // );
        // new RunCommand(
        //     () -> {
        //         drivetrain.applyRequest(() ->
        //                     align.withVelocityX(-m_driverController.getLeftX() * MaxSpeed) // Drive forward with negative Y (forward)
        //                         .withVelocityY(-m_driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
        //                         .withTargetDirection(
        //                             
        //                         ));
        //         }, drivetrain
        //     )
        // );

    m_driverController.leftTrigger().whileTrue(
    new ParallelCommandGroup(
        new Feed(m_shooterSubsystem, m_kickerSubsystem, m_feederSubsystem)
        // new Shoot(m_shooterSubsystem, m_kickerSubsystem, m_feederSubsystem)
    ));
    m_driverController.a().whileTrue(
    new ParallelCommandGroup(
        drivetrain.applyRequest(() ->
                drive.withVelocityX(-Math.sin(time) / 2.0) // Drive forward with negative Y (forward)
                    .withVelocityY(-Math.cos(time) / 2.0)) // Drive left with negative X (left)
    ));

    m_driverController.y()
        .whileTrue(new RunCommand(() -> {IntakeSubsystem.getInstance().spinIntake();}))
        .onFalse(new RunCommand(() -> {IntakeSubsystem.getInstance().stopIntake();}));
    m_driverController.x().whileTrue(new RunCommand(() -> {HoodSubsystem.getInstance().setPercent(1);}));

    //Intake buttons
    m_coDriverController.leftBumper().whileTrue(new InstantCommand(() -> {m_intakeSubsystem.spinIntake();}))
        .onFalse(new InstantCommand(() -> {m_intakeSubsystem.stopIntake();}));
    // m_driverController.rightBumper().whileTrue(new InstantCommand(() -> {m_intakeSubsystem.retract();}))
    //     .onFalse(new InstantCommand(() -> {m_intakeSubsystem.stop();}));

    m_coDriverController.a().whileTrue(new InstantCommand(() -> {m_intakeSubsystem.flipDown();}))
    .onFalse(new InstantCommand(() -> {m_intakeSubsystem.stopFlip();} ));
    m_coDriverController.x().whileTrue(new InstantCommand(() -> {m_intakeSubsystem.flipUp();}))
    .onFalse(new InstantCommand(() -> {m_intakeSubsystem.stopFlip();} ));
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-m_driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-m_driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-m_driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        m_driverController.back().and(m_driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        m_driverController.back().and(m_driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        m_driverController.start().and(m_driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        m_driverController.start().and(m_driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on the hamburger press.
        m_driverController.button(8).onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(m_telemetry::telemeterize);
    }

    public void periodic() {
        time += 2;
    }

 

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
