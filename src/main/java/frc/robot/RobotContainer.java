package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveCommands;

@Logged
public class RobotContainer {
    public static Swerve SWERVE = Swerve.getInstance();

    private final CommandXboxController driverController = new CommandXboxController(OIConstants.kDriverControllerPort);

    public RobotContainer() {
        SWERVE.setDefaultCommand(SwerveCommands.getFieldRelativeOpenLoopSupplierDriveCommand(
                () -> -driverController.getLeftY()/4,
                () -> -driverController.getLeftX()/4,
                () -> -driverController.getRightX()/8
        ));
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        driverController.x().whileTrue(SwerveCommands.getLockSwerveCommand());
        driverController.y().onTrue(SwerveCommands.getResetHeadingCommand());

//        driverController.a().onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
//        driverController.b().onTrue(new InstantCommand(() -> swerveSubsystem.stopModules()));
    }
}
