package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gg.questnav.questnav.*;
import frc.Telemetry;
import frc.robot.Constants.VisionConstants;;

public class QuestNavSubsystem extends SubsystemBase {
    private static QuestNavSubsystem instance;
    
    Matrix<N3, N1> QUESTNAV_STD_DEVS =
        VecBuilder.fill(
            0.02, // Trust down to 2cm in X direction
            0.02, // Trust down to 2cm in Y direction
            0.035 // Trust down to 2 degrees rotational
        );
    
    QuestNav questNav;
    CommandSwerveDrivetrain m_drive;
    Telemetry m_telemetry = null;
    
    boolean hasBeenReset;

    public static QuestNavSubsystem getInstance() {
        if(instance == null) {
            instance = new QuestNavSubsystem();
        }
        return instance;
    }
    
    public QuestNavSubsystem() {
        m_telemetry = Telemetry.getInstance();
        questNav = new QuestNav();
        m_drive = CommandSwerveDrivetrain.getInstance();
        hasBeenReset = false;
    }

    @Override
    public void periodic() {
        questNav.commandPeriodic();
        
        // Get the latest pose data frames from the Quest
        PoseFrame[] questFrames = questNav.getAllUnreadPoseFrames();

        // Loop over the pose data frames and send them to the pose estimator
        for (PoseFrame questFrame : questFrames) {
            // Make sure the Quest was tracking the pose for this frame
            if (questFrame.isTracking()) {
                // Get the pose of the Quest
                Pose3d questPose = questFrame.questPose3d();
                // Get timestamp for when the data was sent
                double timestamp = questFrame.dataTimestamp();

                // Transform by the mount pose to get your robot pose
                Pose3d robotPose = questPose;//.transformBy(QuestNavConstants.ROBOT_TO_QUEST.inverse());

                // You can put some sort of filtering here if you would like!

                // Add the measurement to our estimator
                m_drive.addVisionMeasurement(robotPose.toPose2d(), timestamp, QUESTNAV_STD_DEVS);
                m_telemetry.telemeterizeQuestNav(robotPose);
            }
        }
    }
}
