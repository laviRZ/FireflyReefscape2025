package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

    private final CommandXboxController driverController = new CommandXboxController(OIConstants.kDriverControllerPort);

    public RobotContainer() {
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> -driverController.getLeftY(),
                () -> driverController.getLeftX(),
                () -> driverController.getRightX(),
                () -> !driverController.getHID().getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        driverController.a().onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
        driverController.b().onTrue(new InstantCommand(() -> swerveSubsystem.stopModules()));
    }
}
