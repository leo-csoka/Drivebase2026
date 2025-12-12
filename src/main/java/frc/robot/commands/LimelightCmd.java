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
        LimelightHelpers.SetRobotOrientation("limelight", -robotYawInDegrees, 0, 0, 0, 0, 0);
        // sets orientation for megatag2
        boolean MegaTagVersion = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight").isMegaTag2;
        if (MegaTagVersion == true) {
            System.out.println("Megatag2"); // do we have megatag2?
        }
        Pose2d botPose = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight").pose;
        double timestamp = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight").timestampSeconds;
        drivetrain.addVisionMeasurement(botPose, timestamp);
        // sends pose to drivetrain for autobuilder
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
