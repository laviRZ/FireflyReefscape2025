package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;

public class SwerveModule {
    private final String moduleName;
    private final TalonFX driveMotor;
    private final SparkMax steerMotor;

    private SwerveModuleState targetState = new SwerveModuleState();

    SwerveModule(SwerveModuleConstants.TestingSwerveModules swerveModuleConstants) {
        final SwerveModuleConstants moduleConstants = swerveModuleConstants.swerveModuleConstants;

        this.moduleName = swerveModuleConstants.name();
        driveMotor = moduleConstants.driveMotor;
        steerMotor = moduleConstants.steerMotor;
    }

    public void setTargetState(SwerveModuleState targetState) {
        this.targetState = targetState;
        this.targetState.optimize(getCurrentAngle());
        setTargetAngle(targetState.angle);
        setTargetVelocity(targetState.speedMetersPerSecond);
    }

    protected void setTargetAngle(Rotation2d rotation2d) {
        steerMotor.getClosedLoopController().setReference(rotation2d.getRotations(), SparkBase.ControlType.kPosition);
    }

    private void setTargetVelocity(double velocity){
        setTargetVelocity(velocity, false);
    }

    private void setTargetVelocity(double velocity, boolean closedLoop) {
        if(closedLoop) setTargetClosedLoopVelocity(velocity);
        else setTargetOpenLoopVelocity(velocity);
    }

    protected void setTargetClosedLoopVelocity(double velocity) {
        // TODO
    }

    protected void setTargetOpenLoopVelocity(double velocity) {
        driveMotor.set(velocity / SwerveModuleConstants.MAX_THEORETICAL_SPEED_METERS_PER_SECOND);
    }

    protected void setBrake(boolean brake) {
        driveMotor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    protected void stop() {
        driveMotor.disable();
        steerMotor.disable();
    }

    protected String getModuleName() {
        return moduleName;
    }

    protected double getDriveDistance() {
        double motorRevolutions = driveMotor.getPosition().getValueAsDouble();
        double wheelRevolutions = motorRevolutions / SwerveModuleConstants.DRIVE_GEAR_RATIO;
        return wheelRevolutions * SwerveModuleConstants.WHEEL_DIAMETER_METERS * Math.PI;
    }

    protected double getCurrentVelocity() {
        double motorRps = driveMotor.getVelocity().getValue().in(Units.RotationsPerSecond);
        double wheelRps = motorRps / SwerveModuleConstants.DRIVE_GEAR_RATIO;
        return wheelRps * SwerveModuleConstants.WHEEL_DIAMETER_METERS * Math.PI;
    }

    protected Rotation2d getCurrentAngle() {
        return Rotation2d.fromRotations(steerMotor.getEncoder().getPosition());
    }

    protected SwerveModulePosition getCurrentPosition() {
        return new SwerveModulePosition(getDriveDistance(), getCurrentAngle());
    }

    protected SwerveModuleState getCurrentState() {
        return new SwerveModuleState(getCurrentVelocity(), getCurrentAngle());
    }

    private SwerveModuleState getTargetState() {
        return targetState;
    }
}
