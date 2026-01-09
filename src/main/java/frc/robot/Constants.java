package frc.robot;

import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.robot.generated.TunerConstants;

public class Constants {

      // PathPlanner config constants
        private static final Mass ROBOT_MASS = Kilogram.of(74.088);
        // 15.2
        private static final MomentOfInertia ROBOT_MOI = KilogramSquareMeters.of(6.883);
        private static final double WHEEL_COF = 1.2;
        public static final SwerveModuleConstants<
                TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            SWERVE_MODULE_CONSTANTS = TunerConstants.FrontLeft;
        public static final Translation2d[] SWERVE_MODULE_OFFSETS =
        new Translation2d[] {
        new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
        new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
        new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
        new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
      };

     public static final RobotConfig PP_CONFIG =
      new RobotConfig(
          ROBOT_MASS,
          ROBOT_MOI,
          new ModuleConfig(
              SWERVE_MODULE_CONSTANTS.WheelRadius,
              TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
              WHEEL_COF,
              DCMotor.getKrakenX60Foc(1).withReduction(SWERVE_MODULE_CONSTANTS.DriveMotorGearRatio),
              SWERVE_MODULE_CONSTANTS.SlipCurrent,
              1),
          SWERVE_MODULE_OFFSETS);
}
