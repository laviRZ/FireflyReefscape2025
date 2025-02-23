package frc.robot.subsystems.swerve;

import com.studica.frc.AHRS;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class Swerve extends SubsystemBase {
    private static final Swerve INSTANCE = new Swerve();
    private final SwerveModule[] swerveModules = SwerveConstants.SWERVE_MODULES;

    private Swerve() {
    }

    public static Swerve getInstance() {
        return INSTANCE;
    }

    public double getYaw() {
        return -getGyro().getYaw();
    }

    public void setYaw(double yaw) {
        getGyro().setAngleAdjustment(-yaw);
    }

    protected AHRS getGyro() {
        return SwerveConstants.GYRO;
    }

    protected SwerveModule[] getModules() {
        return swerveModules;
    }

    protected SwerveDriveKinematics getKinematics() {
        return SwerveConstants.KINEMATICS;
    }


    protected void lockSwerve() {
        setBrake(true);
        swerveModules[SwerveModuleConstants.FRONT_LEFT_ID].setTargetState(new SwerveModuleState(0, Rotation2d.fromDegrees(-135)));
        swerveModules[SwerveModuleConstants.FRONT_RIGHT_ID].setTargetState(new SwerveModuleState(0, Rotation2d.fromDegrees(135)));
        swerveModules[SwerveModuleConstants.REAR_LEFT_ID].setTargetState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        swerveModules[SwerveModuleConstants.REAR_RIGHT_ID].setTargetState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }


    /**
     * @return the heading of the robot
     */
    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(MathUtil.inputModulus(getYaw(), -180, 180));
    }
    public void resetGyro() {
        setYaw(0); 
    }
    

    /**
     * @return the robot's current velocity
     */
    public ChassisSpeeds getCurrentVelocity() {
        final SwerveModuleState[] states = new SwerveModuleState[getModules().length];

        for (int i = 0; i < getModules().length; i++)
            states[i] = getModules()[i].getCurrentState();

        return getKinematics().toChassisSpeeds(states);
    }

    /**
     * Sets the heading of the robot.
     *
     * @param heading the new heading
     */
    public void setHeading(Rotation2d heading) {
        setYaw(heading.getDegrees());
    }


    /**
     * Drives the swerve with the given velocities, relative to the robot's frame of reference.
     *
     * @param translation the target x and y velocities in m/s
     * @param rotation    the target theta velocity in radians per second
     */
    protected void selfRelativeDrive(Translation2d translation, Rotation2d rotation) {
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
                translation.getX(),
                translation.getY(),
                rotation.getRadians()
        );
        selfRelativeDrive(chassisSpeeds);
    }

    /**
     * Drives the swerve with the given velocities, relative to the field's frame of reference.
     *
     * @param translation the target x and y velocities in m/s
     * @param rotation    the target theta velocity in radians per second
     */
    protected void fieldRelativeDrive(Translation2d translation, Rotation2d rotation) {
        final Rotation2d heading = getHeading();

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(),
                translation.getY(),
                rotation.getRadians(),
                heading
        );
        selfRelativeDrive(chassisSpeeds);
    }

    /**
     * @return the swerve's module's positions
     */
    protected SwerveModulePosition[] getModulePositions() {
        final SwerveModulePosition[] swerveModuleStates = new SwerveModulePosition[4];
        final SwerveModule[] swerveModules = getModules();

        for (int i = 0; i < swerveModules.length; i++)
            swerveModuleStates[i] = swerveModules[i].getCurrentPosition();

        return swerveModuleStates;
    }

    /**
     * Stops the swerve's motors.
     */
    protected void stop() {
        for (SwerveModule module : getModules())
            module.stop();
    }

    /**
     * Sets whether the drive motors should brake or coast.
     *
     * @param brake whether the drive motors should brake or coast
     */
    protected void setBrake(boolean brake) {
        for (SwerveModule module : getModules())
            module.setBrake(brake);
    }

    /**
     * Sets the swerve's target module states.
     *
     * @param swerveModuleStates the target module states
     */
    protected void setTargetModuleStates(SwerveModuleState[] swerveModuleStates) {
        for (int i = 0; i < getModules().length; i++)
            getModules()[i].setTargetState(swerveModuleStates[i]);
    }

    private void selfRelativeDrive(ChassisSpeeds chassisSpeeds) {
        if (isStill(chassisSpeeds)) {
            stop();
            return;
        }

        SwerveModuleState[] swerveModuleStates = getKinematics().toSwerveModuleStates(chassisSpeeds);
        setTargetModuleStates(swerveModuleStates);
    }

    protected double getDriveNeutralDeadband() {
        return SwerveConstants.DRIVE_NEUTRAL_DEADBAND;
    }

    protected double getRotationNeutralDeadband() {
        return SwerveConstants.ROTATION_NEUTRAL_DEADBAND;
    }

    protected double getTranslationVelocityTolerance() {
        return SwerveConstants.TRANSLATION_VELOCITY_TOLERANCE;
    }

    protected double getRotationVelocityTolerance() {
        return SwerveConstants.ROTATION_VELOCITY_TOLERANCE;
    }

    protected double getMaxSpeedMetersPerSecond() {
        return SwerveConstants.MAX_SPEED_METERS_PER_SECOND;
    }

    protected double getMaxRotationalSpeedRadiansPerSecond() {
        return SwerveConstants.MAX_ROTATIONAL_SPEED_RADIANS_PER_SECOND;
    }

    /**
     * Returns whether the given chassis speeds are considered to be "still" by the swerve neutral deadband.
     *
     * @param chassisSpeeds the chassis speeds to check
     * @return true if the chassis speeds are considered to be "still"
     */
    private boolean isStill(ChassisSpeeds chassisSpeeds) {
        return
                Math.abs(chassisSpeeds.vxMetersPerSecond) <= getDriveNeutralDeadband() &&
                        Math.abs(chassisSpeeds.vyMetersPerSecond) <= getDriveNeutralDeadband() &&
                        Math.abs(chassisSpeeds.omegaRadiansPerSecond) <= getRotationNeutralDeadband();
    }

    public SwerveModuleState[] getCurrentSwerveModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[getModules().length];
        for(int i = 0; i < getModules().length; i++)
            states[i] = getModules()[i].getCurrentState();
        return states;
    }

    public SwerveModuleState[] getTargetSwerveModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[getModules().length];
        for(int i = 0; i < getModules().length; i++)
            states[i] = getModules()[i].getTargetState();
        return states;
    }
    
    @Override
    public void periodic() {
        super.periodic();
    }
}

