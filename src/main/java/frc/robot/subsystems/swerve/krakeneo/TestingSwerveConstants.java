package frc.robot.subsystems.swerve.krakeneo;

import com.pathplanner.lib.config.PIDConstants;
import com.studica.frc.AHRS;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class TestingSwerveConstants {
    static final double BRAKE_TIME_SECONDS = 0.3;
    static final double
            MAX_SPEED_METERS_PER_SECOND = 4.25,
            MAX_ROTATIONAL_SPEED_RADIANS_PER_SECOND = 12.03;
    static final double
            DRIVE_NEUTRAL_DEADBAND = 0.2,
            ROTATION_NEUTRAL_DEADBAND = 0.5;
    static final double
            SIDE_LENGTH_METERS = Units.inchesToMeters(11.3646157) * 2,
            DISTANCE_FROM_CENTER_OF_BASE = SIDE_LENGTH_METERS / 2;
    private static final double RATE_LIMIT = 5.5;
    static final SlewRateLimiter
            X_SLEW_RATE_LIMITER = new SlewRateLimiter(RATE_LIMIT),
            Y_SLEW_RATE_LIMITER = new SlewRateLimiter(RATE_LIMIT);
    private static final Translation2d[] LOCATIONS = {
            TestingSwerveModuleConstants.TestingSwerveModules.fromId(0).location,
            TestingSwerveModuleConstants.TestingSwerveModules.fromId(1).location,
            TestingSwerveModuleConstants.TestingSwerveModules.fromId(2).location,
            TestingSwerveModuleConstants.TestingSwerveModules.fromId(3).location
    };
    static final TestingSwerveModule[] SWERVE_MODULES = {
            new TestingSwerveModule(TestingSwerveModuleConstants.TestingSwerveModules.fromId(0)),
            new TestingSwerveModule(TestingSwerveModuleConstants.TestingSwerveModules.fromId(1)),
            new TestingSwerveModule(TestingSwerveModuleConstants.TestingSwerveModules.fromId(2)),
            new TestingSwerveModule(TestingSwerveModuleConstants.TestingSwerveModules.fromId(3))
    };
    static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(LOCATIONS);
    static final PIDConstants
            TRANSLATION_PID_CONSTANTS = new PIDConstants(12, 0, 0),
            ROTATION_PID_CONSTANTS = new PIDConstants(15, 0, 0),
    AUTO_ROTATION_PID_CONSTANTS = new PIDConstants(15, 0, 0);
    static final AHRS GYRO = new AHRS(AHRS.NavXComType.kMXP_SPI);
    private static final TrapezoidProfile.Constraints ROTATION_CONSTRAINTS = new TrapezoidProfile.Constraints(
            720,
            1200
    );
    static final ProfiledPIDController ROTATION_CONTROLLER = new ProfiledPIDController(
            10,
            0,
            0,
            ROTATION_CONSTRAINTS
    );
    static final double
            TRANSLATION_TOLERANCE = 0.2,
            ROTATION_TOLERANCE = 1,
            TRANSLATION_VELOCITY_TOLERANCE = 0.05,
            ROTATION_VELOCITY_TOLERANCE = 0.05;

    static {
        ROTATION_CONTROLLER.enableContinuousInput(-180, 180);
//        GYRO.configFactoryDefault();
//
//        GYRO.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_1_General, 200);
//        GYRO.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_2_GeneralCompass, 1000);
//        GYRO.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_3_GeneralAccel, 1000);
//        GYRO.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_6_SensorFusion, 1000);
//        GYRO.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_10_SixDeg_Quat, 1000);
//        GYRO.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_11_GyroAccum, 1000);
//        GYRO.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_2_Gyro, 1000);
//        GYRO.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_4_Mag, 1000);
//        GYRO.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_6_Accel, 1000);
    }
}
