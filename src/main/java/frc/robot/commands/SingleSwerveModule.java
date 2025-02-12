package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.Supplier;

public class SingleSwerveModule extends Command {
    private final SwerveModule swerveModule;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> fieldOrientedFunction;


    public SingleSwerveModule(SwerveModule swerveModule,
                              Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
                              Supplier<Boolean> fieldOrientedFunction) {
        this.swerveModule = swerveModule;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
//        addRequirements(swerveModule);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // 1. Get real-time joystick inputs
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        double turningSpeed = turningSpdFunction.get();

        // 2. Apply deadband
        xSpeed = Math.abs(xSpeed) > 0.1 ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > 0.1 ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > 0.1 ? turningSpeed : 0.0;

        // 3. Make the driving smoother
        xSpeed = xSpeed * 1.0;
        ySpeed = ySpeed * 1.0;
        turningSpeed = turningSpeed * 1.0;

        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        if (fieldOrientedFunction.get()) {
            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, turningSpeed, swerveModule.getRotation2d());
        } else {
            // Relative to robot
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // 6. Set module states
        swerveModule.setDesiredState(moduleStates[0]);

    }

    @Override
    public void end(boolean interrupted) {
        swerveModule.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
