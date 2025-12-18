package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class LimelightSubsystem extends SubsystemBase{

    private final CommandSwerveDrivetrain drivetrain;

    public LimelightSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    public void zeroPigeonYaw() {
        drivetrain.getPigeon2().setYaw(0);
    }

    @Override
    public void periodic() {
        double robotYawInDegrees = drivetrain.getPigeon2().getRotation2d().getDegrees();
        LimelightHelpers.SetRobotOrientation("limelight", robotYawInDegrees, 0, 0, 0, 0, 0);
        //System.out.println(robotYawInDegrees);
        // sets orientation for megatag2
        LimelightHelpers.PoseEstimate current_estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
 
        Pose2d botPose = current_estimate.pose;
        double timestamp = current_estimate.timestampSeconds;
        // drivetrain.addVisionMeasurement(botPose, timestamp);
        // sends pose to drivetrain for autobuilder
    }
}
