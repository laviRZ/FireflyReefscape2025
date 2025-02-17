//package frc.robot;
//
//import java.util.List;
//
//import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.math.controller.ProfiledPIDController;
//import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.geometry.Translation2d;
//import edu.wpi.first.math.trajectory.Trajectory;
//import edu.wpi.first.math.trajectory.TrajectoryConfig;
//import edu.wpi.first.math.trajectory.TrajectoryGenerator;
//import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
//import frc.robot.Constants.AutoConstants;
//import frc.robot.Constants.DriveConstants;
//import frc.robot.Constants.OIConstants;
//import frc.robot.commands.SwerveJoystickCmd;
//import frc.robot.subsystems.SwerveSubsystem;
//
//public class RobotContainer {
//
//    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
//
//    private final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);
//
//    public RobotContainer() {
//        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
//                swerveSubsystem,
//                () -> -driverJoytick.getRawAxis(OIConstants.kDriverYAxis),
//                () -> driverJoytick.getRawAxis(OIConstants.kDriverXAxis),
//                () -> driverJoytick.getRawAxis(OIConstants.kDriverRotAxis),
//                () -> !driverJoytick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));
//
//        configureButtonBindings();
//    }
//
//    private void configureButtonBindings() {
//        //new JoystickButton(driverJoytick, 2).whenPressed(() -> swerveSubsystem.zeroHeading());
//    }
//
//    public Command getAutonomousCommand() {
//        // 1. Create trajectory settings
//        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
//                AutoConstants.kMaxSpeedMetersPerSecond,
//                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
//                        .setKinematics(DriveConstants.kDriveKinematics);
//
//        // 2. Generate trajectory
//        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
//                new Pose2d(0, 0, new Rotation2d(0)),
//                List.of(
//                        new Translation2d(1, 0),
//                        new Translation2d(1, -1)),
//                new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
//                trajectoryConfig);
//
//        // 3. Define PID controllers for tracking trajectory
//        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
//        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
//        ProfiledPIDController thetaController = new ProfiledPIDController(
//                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
//        thetaController.enableContinuousInput(-Math.PI, Math.PI);
//
//        // 4. Construct command to follow trajectory
//        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
//                trajectory,
//                swerveSubsystem::getPose,
//                DriveConstants.kDriveKinematics,
//                xController,
//                yController,
//                thetaController,
//                swerveSubsystem::setModuleStates,
//                swerveSubsystem);
//
//        // 5. Add some init and wrap-up, and return everything
//        return new SequentialCommandGroup(
//                new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
//                swerveControllerCommand,
//                new InstantCommand(() -> swerveSubsystem.stopModules()));
//    }
//}


package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.output.OutputCommands;
import frc.robot.subsystems.output.OutputSubsystem;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

import java.io.File;
import java.io.IOException;
import java.util.function.DoubleSupplier;

import static edu.wpi.first.wpilibj2.command.Commands.run;

public class RobotContainer {

    public static final OutputSubsystem outputSubsystem = new OutputSubsystem();
    final SwerveDrive swerveDrive;
    private final CommandXboxController driverController = new CommandXboxController(OIConstants.kDriverControllerPort);

    public RobotContainer() {
        double maximumSpeed = Units.feetToMeters(4.5);
        File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
        try {
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
        } catch(IOException e) {
            throw new RuntimeException(e);
        }
        SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;

        var driveCmd = driveCommand(
                () -> -driverController.getLeftX(),
                () -> -driverController.getLeftY(),
                () -> -driverController.getRightX()
        );
        driveCmd.schedule();

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        driverController.y().whileTrue(OutputCommands.outputL4());
        driverController.x().whileTrue(OutputCommands.outputL2L3());
        driverController.a().whileTrue(OutputCommands.stop());
        driverController.b().whileTrue(OutputCommands.outputStraight());

//        driverController.a().onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
//        driverController.b().onTrue(new InstantCommand(() -> swerveSubsystem.stopModules()));
    }


    /**
     * Command to drive the robot using translative values and heading as angular velocity.
     *
     * @param translationX     Translation in the X direction.
     * @param translationY     Translation in the Y direction.
     * @param angularRotationX Rotation of the robot to set
     * @return Drive command.
     */
    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
    {
        return run(() -> {
            // Make the robot move
            swerveDrive.drive(new Translation2d(translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                            translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()),
                    angularRotationX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(),
                    true,
                    true);
        });
    }
}
