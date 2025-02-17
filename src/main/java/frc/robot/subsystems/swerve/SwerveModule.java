package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
    private final TalonFX driveMotor;
    private final SparkMax steerMotor;
    private final CANcoder steerEncoder;
    private final String moduleName;
    private Rotation2d targetAngle = Rotation2d.fromDegrees(0);

    SwerveModule(SwerveModuleConstants.TestingSwerveModules swerveModule) {
        final SwerveModuleConstants moduleConstants = swerveModule.swerveModuleConstants;

        driveMotor = moduleConstants.driveMotor;
        steerMotor = moduleConstants.steerMotor;
        steerEncoder = moduleConstants.steerEncoder;
        this.moduleName = swerveModule.name();
    }

    protected Rotation2d getTargetAngle() {
        return targetAngle;
    }

    protected void setBrake(boolean brake) {
        driveMotor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    protected void stop() {
        driveMotor.disable();
        steerMotor.disable();
    }

    protected void setTargetClosedLoopVelocity(double velocity) {
//        final double driveMotorVelocity = Conversions.systemToMotor(velocity, TestingSwerveModuleConstants.DRIVE_GEAR_RATIO);
//        final double feedForward = TestingSwerveModuleConstants.DRIVE_FEEDFORWARD.calculate(driveMotorVelocity);
//
//        driveMotor.setControl(
//                ControlModeValue.VelocityVoltage, driveMotorVelocity,
//                feedForward, feedForward
//        );
    }

    protected void setTargetOpenLoopVelocity(double velocity) {
        double power = velocity / SwerveModuleConstants.MAX_THEORETICAL_SPEED_METERS_PER_SECOND;
        driveMotor.set(power);
    }

    protected String getModuleName() {
        return moduleName;
    }

    protected void setTargetAngle(Rotation2d rotation2d) {
        targetAngle = rotation2d;
        SmartDashboard.putNumber("swerve/" + getModuleName() + " tangle", rotation2d.getRotations());

//        steerMotor.getPIDController().setReference(rotation2d.getDegrees(), ControlType.kPosition);
        steerMotor.getClosedLoopController().setReference(rotation2d.getRotations(), SparkBase.ControlType.kPosition);
    }

    protected double getDriveDistance() {
        double motorRevolutions = driveMotor.getPosition().getValueAsDouble();
        double wheelRevolutions = motorRevolutions / SwerveModuleConstants.DRIVE_GEAR_RATIO;
        return wheelRevolutions * SwerveModuleConstants.WHEEL_DIAMETER_METERS * Math.PI;
    }

    protected SwerveModuleState optimizeState(SwerveModuleState state) {
        return SwerveModuleState.optimize(state, getCurrentAngle());
    }

    protected double getCurrentVelocity() {
        double motorRps = driveMotor.getVelocity().getValue().in(Units.RotationsPerSecond);
        double wheelRps = motorRps / SwerveModuleConstants.DRIVE_GEAR_RATIO;
        return wheelRps * SwerveModuleConstants.WHEEL_DIAMETER_METERS * Math.PI;
    }

    protected Rotation2d getCurrentAngle() {
        return Rotation2d.fromRotations(steerMotor.getEncoder().getPosition());
    }

    private SwerveModuleState targetState = new SwerveModuleState();
    private boolean driveMotorClosedLoop = false;
    private double targetVelocity = 0;

    /**
     * @return the current position of the module
     */
    protected SwerveModulePosition getCurrentPosition() {
        return new SwerveModulePosition(getDriveDistance(), getCurrentAngle());
    }

    /**
     * @return the current state of the module
     */
    protected SwerveModuleState getCurrentState() {
        return new SwerveModuleState(getCurrentVelocity(), getCurrentAngle());
    }

    /**
     * Sets the target state for the module.
     *
     * @param targetState the target state
     */
    public void setTargetState(SwerveModuleState targetState) {
        this.targetState = targetState = optimizeState(targetState);
        setTargetAngle(targetState.angle);
        setTargetVelocity(targetState.speedMetersPerSecond);
    }

    /**
     * Sets whether the drive motor should be in closed loop control, or in open loop control.
     *
     * @param closedLoop true if the drive motor should be in closed loop control, false if it should be in open loop control
     */
    protected void setDriveMotorClosedLoop(boolean closedLoop) {
        driveMotorClosedLoop = closedLoop;
    }

    /**
     * @return the target state of the module
     */
    private SwerveModuleState getTargetState() {
        return targetState;
    }


    /**
     * @return the target velocity of the module
     */
    private double getTargetVelocity() {
        return targetVelocity;
    }

    /**
     * Sets the target velocity for the module.
     *
     * @param velocity the target velocity
     */
    private void setTargetVelocity(double velocity) {
        targetVelocity = velocity;
        if (driveMotorClosedLoop)
            setTargetClosedLoopVelocity(velocity);
        else
            setTargetOpenLoopVelocity(velocity);
    }

    private void setTargetDegrees(double degrees) {
        setTargetAngle(Rotation2d.fromDegrees(degrees));
    }
}

