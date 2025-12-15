package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain; 


public class LimelightCmd extends Command {

    private final CommandSwerveDrivetrain drivetrain;

    public LimelightCmd(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void execute() {
        double robotYawInDegrees = drivetrain.getPigeon2().getRotation2d().getDegrees();
        LimelightHelpers.SetRobotOrientation("limelight", robotYawInDegrees, 0, 0, 0, 0, 0);
        //System.out.println(robotYawInDegrees);
        // sets orientation for megatag2
        LimelightHelpers.PoseEstimate current_estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
 
        Pose2d botPose = current_estimate.pose;
        double timestamp = current_estimate.timestampSeconds;
        drivetrain.addVisionMeasurement(botPose, timestamp);
        // sends pose to drivetrain for autobuilder
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
