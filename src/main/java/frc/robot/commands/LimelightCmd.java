package frc.robot.commands;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain; 


public class LimelightCmd extends Command {
    public final CANBus canivore = new CANBus("drivetrain");
    private final Pigeon2 pigeon = new Pigeon2(0, canivore);

    private final CommandSwerveDrivetrain drivetrain;

    public LimelightCmd(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void execute() {
        double robotYawInDegrees = pigeon.getRotation2d().getDegrees();
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
