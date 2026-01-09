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
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElasticSubsystem;

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
    private final ElasticSubsystem elasticSubsystem = new ElasticSubsystem();
    // private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem(drivetrain);

    public boolean isFollowingPath = false;


    private final Trigger a = controller.a();
    private final Trigger b = controller.b();
    private final Trigger x = controller.x();
    private final Trigger y = controller.y();

    private SendableChooser<Command> autoChooser;

    public RobotContainer() {
        isFollowingPath = false;
        autoChooser = AutoBuilder.buildAutoChooser("Example");
        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        FollowPathCommand.warmupCommand().schedule();
    }          

    double kP_angle = 5;
    double currentTA = 0;
    double currentTX = 0;

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

    public Command GeneratePath(Pose2d targetPose) {

        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
                0.5, 0.5,
                Units.degreesToRadians(540), Units.degreesToRadians(720));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand = AutoBuilder.pathfindToPose(
        targetPose,
        constraints,
        0.0 // Goal end velocity in meters/sec        
        );
        return pathfindingCommand;
    }    

    
    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

        a.onTrue(Commands.runOnce(()-> {
            currentTA = LimelightHelpers.getTA("limelight"); 
            if (!isFollowingPath) {
                isFollowingPath = true;
                Pose2d target = new Pose2d(4, 2, Rotation2d.fromDegrees(0));
                Command limelightPath = GeneratePath(target);

                limelightPath.andThen(() -> isFollowingPath = false).schedule();
            }
        })
    );

    drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                double vy = 0;
                double vx = 0;
                double angle = 0;
                
                currentTA = LimelightHelpers.getTA("limelight");
                currentTX = LimelightHelpers.getTX("limelight");             
                
                if (x.getAsBoolean() && !isFollowingPath) {   
                    vy = 0;
                    vx = LimelightTranslation(currentTA) * -1;
                    angle = currentTX * kP_angle * -1;
                    System.out.println(angle);              
                    return driveRobotOriented.withVelocityX(vx)
                                .withVelocityY(0)
                                .withRotationalRate(Math.toRadians(angle));
                } else if (!isFollowingPath) {
                    vx = (-controller.getLeftY() * MaxSpeed) * 0.5;
                    vy = (-controller.getLeftX() * MaxSpeed) * 0.5;
                    angle = (-controller.getRightX() * MaxAngularRate);
                    return drive.withVelocityX(vx)
                                .withVelocityY(vy)
                                .withRotationalRate((angle));
                } else {
                    return drive.withVelocityX(0)
                    .withVelocityY(0)
                    .withRotationalRate((0));
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

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // controller.back().and(controller.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // controller.back().and(controller.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // controller.start().and(controller.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // controller.start().and(controller.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        controller.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}