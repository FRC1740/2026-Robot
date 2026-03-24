// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

import java.util.List;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/** Add your docs here. */
public class Telemetry {
    private static Telemetry instance;
    NetworkTableInstance ins = NetworkTableInstance.getDefault();
    NetworkTable shooterTable = ins.getTable("Shooter table");
    NetworkTable intakeTable = ins.getTable("Intake table");
    NetworkTable kickerTable = ins.getTable("Kicker table");

    DoublePublisher flywheelRPM = shooterTable.getDoubleTopic("Flywheel rpm").publish();
    DoublePublisher flywheelCurrentDrawL = shooterTable.getDoubleTopic("Flywheel Current Draw L").publish();
    DoublePublisher flywheelCurrentDrawR = shooterTable.getDoubleTopic("Flywheel Current Draw R").publish();

    DoublePublisher intakeDist = intakeTable.getDoubleTopic("intake dist").publish();
    BooleanPublisher intakeEjecting = intakeTable.getBooleanTopic("intake ejecting").publish();
    DoublePublisher intakeStallTime = intakeTable.getDoubleTopic("intake stall time").publish();
    DoublePublisher intakeRollersCurrentDraw = intakeTable.getDoubleTopic("intake rollers current draw").publish();
    DoublePublisher intakeRollersSpeed = intakeTable.getDoubleTopic("intake rollers speed").publish();
    DoublePublisher intakeFlipCurrentDraw = intakeTable.getDoubleTopic("intake flip current draw").publish();
    DoublePublisher kickerRPM = intakeTable.getDoubleTopic("kicker rpm").publish();
    DoublePublisher kickerCurrentDraw = intakeTable.getDoubleTopic("kicker current draw").publish();
    DoublePublisher feederCurrentDraw = intakeTable.getDoubleTopic("feeder current draw").publish();

    private final double MaxSpeed = 6.0;

    /**
     * Construct a telemetry object, with the specified max speed of the robot
     * 
     * @param maxSpeed Maximum speed in meters per second
     */
    public Telemetry() {
        SignalLogger.start();

        /* Set up the module state Mechanism2d telemetry */
        for (int i = 0; i < 4; ++i) {
            SmartDashboard.putData("Module " + i, m_moduleMechanisms[i]);
        }
    }

    /* What to publish over networktables for telemetry */
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    /* Robot swerve drive state */
    private final NetworkTable driveStateTable = inst.getTable("DriveState");
    private final StructPublisher<Pose2d> drivePose = driveStateTable.getStructTopic("Pose", Pose2d.struct).publish();
    private final StructPublisher<ChassisSpeeds> driveSpeeds = driveStateTable.getStructTopic("Speeds", ChassisSpeeds.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleStates = driveStateTable.getStructArrayTopic("ModuleStates", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleTargets = driveStateTable.getStructArrayTopic("ModuleTargets", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModulePosition> driveModulePositions = driveStateTable.getStructArrayTopic("ModulePositions", SwerveModulePosition.struct).publish();
    private final DoublePublisher driveTimestamp = driveStateTable.getDoubleTopic("Timestamp").publish();
    private final DoublePublisher driveOdometryFrequency = driveStateTable.getDoubleTopic("OdometryFrequency").publish();
    private final BooleanPublisher driveFlipped = driveStateTable.getBooleanTopic("FlippedControl").publish();


    private final NetworkTable matchStateTable = inst.getTable("MatchState");
    private final DoublePublisher matchTime = matchStateTable.getDoubleTopic("time remaining").publish();
    private final DoublePublisher shiftTime = matchStateTable.getDoubleTopic("time remaining in shift").publish();

    /* Robot pose for field positioning */
    private final NetworkTable table = inst.getTable("Pose");
    private final DoubleArrayPublisher fieldPub = table.getDoubleArrayTopic("robotPose").publish();
    private final StringPublisher fieldTypePub = table.getStringTopic(".type").publish();

    private final StructArrayPublisher<Pose2d> autoFieldPub = table.getStructArrayTopic("robotAutoPose", Pose2d.struct).publish();


    /* Mechanisms to represent the swerve module states */
    private final Mechanism2d[] m_moduleMechanisms = new Mechanism2d[] {
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
    };
    /* A direction and length changing ligament for speed representation */
    private final MechanismLigament2d[] m_moduleSpeeds = new MechanismLigament2d[] {
        m_moduleMechanisms[0].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[1].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[2].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[3].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
    };
    /* A direction changing and length constant ligament for module direction */
    private final MechanismLigament2d[] m_moduleDirections = new MechanismLigament2d[] {
        m_moduleMechanisms[0].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[1].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[2].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[3].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
    };

    private final double[] m_poseArray = new double[3];

    private final StructPublisher<Pose2d> questPose = driveStateTable.getStructTopic("QuestPose", Pose2d.struct).publish();


    /** Accept the swerve drive state and telemeterize it to SmartDashboard and SignalLogger. */
    public void telemeterize(SwerveDriveState state) {
        /* Telemeterize the swerve drive state */
        drivePose.set(state.Pose);
        driveSpeeds.set(state.Speeds);
        driveModuleStates.set(state.ModuleStates);
        driveModuleTargets.set(state.ModuleTargets);
        driveModulePositions.set(state.ModulePositions);
        driveTimestamp.set(state.Timestamp);
        driveOdometryFrequency.set(1.0 / state.OdometryPeriod);

        /* Also write to log file */
        SignalLogger.writeStruct("DriveState/Pose", Pose2d.struct, state.Pose);
        SignalLogger.writeStruct("DriveState/Speeds", ChassisSpeeds.struct, state.Speeds);
        SignalLogger.writeStructArray("DriveState/ModuleStates", SwerveModuleState.struct, state.ModuleStates);
        SignalLogger.writeStructArray("DriveState/ModuleTargets", SwerveModuleState.struct, state.ModuleTargets);
        SignalLogger.writeStructArray("DriveState/ModulePositions", SwerveModulePosition.struct, state.ModulePositions);
        SignalLogger.writeDouble("DriveState/OdometryPeriod", state.OdometryPeriod, "seconds");

        /* Telemeterize the pose to a Field2d */
        fieldTypePub.set("Field2d");

        m_poseArray[0] = state.Pose.getX();
        m_poseArray[1] = state.Pose.getY();
        m_poseArray[2] = state.Pose.getRotation().getDegrees();
        fieldPub.set(m_poseArray);

        /* Telemeterize each module state to a Mechanism2d */
        for (int i = 0; i < 4; ++i) {
            m_moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
            m_moduleDirections[i].setAngle(state.ModuleStates[i].angle);
            m_moduleSpeeds[i].setLength(state.ModuleStates[i].speedMetersPerSecond / (2 * MaxSpeed));
        }
    }

    public void setMatchTime() {
        matchTime.set(DriverStation.getMatchTime());
        double time = DriverStation.getMatchTime();
        if (time > (2*60) + 10) {
            shiftTime.set(time - ((2*60) + 10));
        }else if (time > 60 + 45) {
            shiftTime.set(time - (60 + 45));
        }else if (time > 60 + 20) {
            shiftTime.set(time - (60 + 20));
        }else if (time > 55) {
            shiftTime.set(time - (55));
        }else if (time > 30) {
            shiftTime.set(time - (30));
        }else {
            shiftTime.set(time);
        }
    }

    public void setAutoPath(Pose2d[] poses) {
        autoFieldPub.set(poses);
    }
    
    public static Telemetry getInstance() {
        if(instance == null) {
            instance = new Telemetry();
        }
        return instance;
    }

    public void telemetrizeShooter(double rpm, double leftCurrentDraw, double rightCurrentDraw) {
        flywheelRPM.set(rpm);
        flywheelCurrentDrawL.set(leftCurrentDraw);
        flywheelCurrentDrawR.set(rightCurrentDraw);
        driveFlipped.set(CommandSwerveDrivetrain.getInstance().m_operatorPerspectiveFlipped);
    }

    public void telemetrizeFeeder(double currentDraw) {
        feederCurrentDraw.set(currentDraw);
    }

    public void telemetrizeKicker(double rpm, double currentDraw) {
        kickerRPM.set(rpm);
        kickerCurrentDraw.set(currentDraw);
    }

    public void telemetrizeIntake(double dist, double currentDrawRollers, double currentDrawFlip, double rollerSpeed, boolean ejecting, double stallTime) {
        intakeDist.set(dist);
        intakeRollersCurrentDraw.set(currentDrawRollers);
        intakeFlipCurrentDraw.set(currentDrawFlip);
        intakeRollersSpeed.set(rollerSpeed);
        intakeEjecting.set(ejecting);
        intakeStallTime.set(stallTime);
    }
    
    public void telemeterizeQuestNav(Pose3d robotPose) {
        this.questPose.set(robotPose.toPose2d());
    }
}
