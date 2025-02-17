package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public abstract class SwerveModule  {
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
     * @return the target angle of the module
     */
    protected abstract Rotation2d getTargetAngle();

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

    /**
     * Sets whether the drive motor should brake or coast
     *
     * @param brake true if the drive motor should brake, false if it should coast
     */
    protected abstract void setBrake(boolean brake);

    /**
     * Stops the module from moving.
     */
    protected abstract void stop();

    /**
     * @return the module's current angle
     */
    protected abstract Rotation2d getCurrentAngle();

    /**
     * @return the module's current velocity in meters per second
     */
    protected abstract double getCurrentVelocity();

    /**
     * @return the module's current drive distance in meters
     */
    protected abstract double getDriveDistance();

    /**
     * Sets the module's target angle.
     *
     * @param rotation2d the target angle
     */
    protected abstract void setTargetAngle(Rotation2d rotation2d);

    /**
     * Optimizes the module state.
     *
     * @param state the state to optimize
     * @return the optimized state
     */
    protected abstract SwerveModuleState optimizeState(SwerveModuleState state);

    /**
     * Sets the module's target velocity in closed loop control.
     *
     * @param velocity the target velocity
     */
    protected abstract void setTargetClosedLoopVelocity(double velocity);

    /**
     * Sets the module's target velocity in open loop control.
     *
     * @param velocity the target velocity
     */
    protected abstract void setTargetOpenLoopVelocity(double velocity);

    /**
     * @return the module's name
     */
    protected abstract String getModuleName();
}
