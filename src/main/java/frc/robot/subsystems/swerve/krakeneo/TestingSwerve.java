package frc.robot.subsystems.swerve.krakeneo;

import com.pathplanner.lib.config.PIDConstants;
import com.studica.frc.AHRS;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveModule;

public class TestingSwerve extends Swerve {
    private static final TestingSwerve INSTANCE = new TestingSwerve();
    private final TestingSwerveModule[] swerveModules = TestingSwerveConstants.SWERVE_MODULES;

    private TestingSwerve() {
    }

    public static TestingSwerve getInstance() {
        return INSTANCE;
    }

    @Override
    public double getPitch() {
        return getGyro().getPitch();
    }

    @Override
    public double getYaw() {
        return getGyro().getYaw();
    }

    @Override
    public void setYaw(double yaw) {
        getGyro().setAngleAdjustment(yaw);
    }

    protected AHRS getGyro() {
        return TestingSwerveConstants.GYRO;
    }

    @Override
    protected SwerveModule[] getModules() {
        return swerveModules;
    }

    @Override
    protected SwerveDriveKinematics getKinematics() {
        return TestingSwerveConstants.KINEMATICS;
    }

    @Override
    protected double getDriveNeutralDeadband() {
        return TestingSwerveConstants.DRIVE_NEUTRAL_DEADBAND;
    }

    @Override
    protected double getRotationNeutralDeadband() {
        return TestingSwerveConstants.ROTATION_NEUTRAL_DEADBAND;
    }

    @Override
    public PIDConstants getTranslationPIDConstants() {
        return TestingSwerveConstants.TRANSLATION_PID_CONSTANTS;
    }

    @Override
    protected PIDConstants getRotationPIDConstants() {
        return TestingSwerveConstants.ROTATION_PID_CONSTANTS;
    }

    @Override
    protected PIDConstants getAutoRotationPIDConstants() {
        return TestingSwerveConstants.AUTO_ROTATION_PID_CONSTANTS;
    }

    @Override
    protected double getMaxSpeedMetersPerSecond() {
        return TestingSwerveConstants.MAX_SPEED_METERS_PER_SECOND;
    }

    @Override
    protected double getMaxRotationalSpeedRadiansPerSecond() {
        return TestingSwerveConstants.MAX_ROTATIONAL_SPEED_RADIANS_PER_SECOND;
    }

    @Override
    protected double getBrakeTimeSeconds() {
        return TestingSwerveConstants.BRAKE_TIME_SECONDS;
    }

    @Override
    public ProfiledPIDController getRotationController() {
        return TestingSwerveConstants.ROTATION_CONTROLLER;
    }

    @Override
    protected void lockSwerve() {
        setBrake(true);
        swerveModules[TestingSwerveModuleConstants.FRONT_LEFT_ID].setTargetAngle(Rotation2d.fromDegrees(45));
        swerveModules[TestingSwerveModuleConstants.FRONT_RIGHT_ID].setTargetAngle(Rotation2d.fromDegrees(-45));
        swerveModules[TestingSwerveModuleConstants.REAR_LEFT_ID].setTargetAngle(Rotation2d.fromDegrees(-45));
        swerveModules[TestingSwerveModuleConstants.REAR_RIGHT_ID].setTargetAngle(Rotation2d.fromDegrees(45));
    }

    @Override
    protected double getTranslationTolerance() {
        return TestingSwerveConstants.TRANSLATION_TOLERANCE;
    }

    @Override
    protected double getRotationTolerance() {
        return TestingSwerveConstants.ROTATION_TOLERANCE;
    }

    @Override
    protected double getTranslationVelocityTolerance() {
        return TestingSwerveConstants.TRANSLATION_VELOCITY_TOLERANCE;
    }

    @Override
    protected double getRotationVelocityTolerance() {
        return TestingSwerveConstants.ROTATION_VELOCITY_TOLERANCE;
    }

    @Override
    protected SlewRateLimiter getXSlewRateLimiter() {
        return TestingSwerveConstants.X_SLEW_RATE_LIMITER;
    }

    @Override
    protected SlewRateLimiter getYSlewRateLimiter() {
        return TestingSwerveConstants.Y_SLEW_RATE_LIMITER;
    }
}

