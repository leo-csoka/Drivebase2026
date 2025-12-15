// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.List;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.LimelightCmd;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.RobotCentric driveRobotOriented = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController controller = new CommandXboxController(0);

    public static final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        configureBindings();
    }          

    double kP_angle = 5;
    double currentTA = 0;
    double currentTX = 0;
    private boolean isFollowingPath = false;


    public double LimelightTranslation(double ta) {
        double translation = 0;
        if (ta <= 2.0) {
            translation = -0.5;     
        } else if (ta > 2.0 && ta < 8.0) {
            translation = -0.3;
        } else if (ta >= 8.0 && ta < 15.0) {
            translation = 0.2;
        } else {
            translation = 0;
        }
        return translation;
    }

    public PathPlannerPath GeneratePath(Pose2d startPose, Pose2d endPose) {
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            startPose,
            endPose
        );
    
        PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI);
    
        // Create the path using the waypoints
        PathPlannerPath path = new PathPlannerPath(
            waypoints,
            constraints,
            null,
            new GoalEndState(0.0, endPose.getRotation()) // end rotation and velocity
        );
    
        path.preventFlipping = true;
        return path;
    }    

    
    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

        controller.a().onTrue(Commands.runOnce(()-> {
            currentTA = LimelightHelpers.getTA("limelight");
            if (!isFollowingPath) {
                Pose2d start = LimelightHelpers.getBotPose2d_wpiBlue("limelight");
                Pose2d end = new Pose2d(2.0, 4.0, Rotation2d.fromDegrees(0));
                PathPlannerPath limelightPath = GeneratePath(start, end);

                Command pathCommand = AutoBuilder.followPath(limelightPath)
                .andThen(() -> isFollowingPath = false);

                pathCommand.schedule(); 
                isFollowingPath = true;
            }
        })
    );

    drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                double vx = 0;
                double vy = 0;
                double angle = 0;
                
                currentTA = LimelightHelpers.getTA("limelight");
                currentTX = LimelightHelpers.getTX("limelight");             
                
                if (controller.x().getAsBoolean() && !isFollowingPath) {   
                    vx = 0;
                    vy = LimelightTranslation(currentTA) * -1;
                    angle = currentTX * kP_angle * -1;
                    System.out.println(angle);              
                    return driveRobotOriented.withVelocityX(vy)
                                .withVelocityY(0)
                                .withRotationalRate(Math.toRadians(angle));
                } else {
                    vx = (-controller.getLeftY() * MaxSpeed) * 0.5;
                    vy = (-controller.getLeftX() * MaxSpeed) * 0.5;
                    angle = (-controller.getRightX() * MaxAngularRate);
                    return drive.withVelocityX(vx)
                                .withVelocityY(vy)
                                .withRotationalRate((angle));
                }
            })
        );

 // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        //controller.a().whileTrue(drivetrain.applyRequest(() -> brake));
        controller.b().onTrue(new LimelightCmd(drivetrain));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        controller.back().and(controller.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        controller.back().and(controller.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        controller.start().and(controller.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        controller.start().and(controller.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        controller.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return new PathPlannerAuto("example");
    }
}