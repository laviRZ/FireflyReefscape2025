package frc.robot.subsystems.swerve.krakeneo;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.swerve.Conversions;
import frc.robot.subsystems.swerve.SwerveModule;

public class TestingSwerveModule extends SwerveModule {
    private final TalonFX driveMotor;
    private final SparkMax steerMotor;
    private final CANcoder steerEncoder;
    private final String moduleName;
    private Rotation2d targetAngle = Rotation2d.fromDegrees(0);

    TestingSwerveModule(TestingSwerveModuleConstants.TestingSwerveModules swerveModule) {
        final TestingSwerveModuleConstants moduleConstants = swerveModule.swerveModuleConstants;

        driveMotor = moduleConstants.driveMotor;
        steerMotor = moduleConstants.steerMotor;
        steerEncoder = moduleConstants.steerEncoder;
        this.moduleName = swerveModule.name();
    }

    @Override
    protected Rotation2d getTargetAngle() {
        return targetAngle;
    }

    @Override
    protected void setBrake(boolean brake) {
        driveMotor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    @Override
    protected void stop() {
        driveMotor.disable();
        steerMotor.disable();
    }

    @Override
    protected void setTargetClosedLoopVelocity(double velocity) {
//        final double driveMotorVelocity = Conversions.systemToMotor(velocity, TestingSwerveModuleConstants.DRIVE_GEAR_RATIO);
//        final double feedForward = TestingSwerveModuleConstants.DRIVE_FEEDFORWARD.calculate(driveMotorVelocity);
//
//        driveMotor.setControl(
//                ControlModeValue.VelocityVoltage, driveMotorVelocity,
//                feedForward, feedForward
//        );
    }

    @Override
    protected void setTargetOpenLoopVelocity(double velocity) {
        double power = velocity / TestingSwerveModuleConstants.MAX_THEORETICAL_SPEED_METERS_PER_SECOND;
        driveMotor.set(power);
    }

    @Override
    protected String getModuleName() {
        return moduleName;
    }

    @Override
    protected void setTargetAngle(Rotation2d rotation2d) {
        targetAngle = rotation2d;
        SmartDashboard.putNumber("swerve/" + getModuleName() + " tangle", rotation2d.getRotations());

//        steerMotor.getPIDController().setReference(rotation2d.getDegrees(), ControlType.kPosition);
        steerMotor.getClosedLoopController().setReference(rotation2d.getRotations(), SparkBase.ControlType.kPosition);
    }

    @Override
    protected double getDriveDistance() {
        double ticks = driveMotor.getPosition().getValueAsDouble();
        double motorRevolutions = Conversions.falconTicksToRevolutions(ticks);
        double wheelRevolutions = Conversions.motorToSystem(motorRevolutions, TestingSwerveModuleConstants.DRIVE_GEAR_RATIO);
        return Conversions.revolutionsToDistance(wheelRevolutions, TestingSwerveModuleConstants.WHEEL_DIAMETER_METERS);
    }

    @Override
    protected SwerveModuleState optimizeState(SwerveModuleState state) {
        return SwerveModuleState.optimize(state, getCurrentAngle());
    }

    @Override
    protected double getCurrentVelocity() {
//        double motorTicksPer100Ms = driveMotor.getVelocity().getValue().in(AngularVelocityUnit.combine(AngleUnit));
//        double motorRevolutionsPer100Ms = Conversions.falconTicksToRevolutions(motorTicksPer100Ms);
//        double motorRps = Conversions.perHundredMsToPerSecond(motorRevolutionsPer100Ms);
//        double wheelRps = Conversions.motorToSystem(motorRps, TestingSwerveModuleConstants.DRIVE_GEAR_RATIO);
//        return Conversions.revolutionsToDistance(wheelRps, TestingSwerveModuleConstants.WHEEL_DIAMETER_METERS);
        return 0;
    }

    @Override
    protected Rotation2d getCurrentAngle() {
        return Rotation2d.fromRotations(steerMotor.getEncoder().getPosition());
    }
}

