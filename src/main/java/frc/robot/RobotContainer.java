package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Elevator.ElevatorCommands;
import frc.robot.subsystems.Elevator.ElevatorConstants.ElevatorState;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.output.OutputCommands;
import frc.robot.subsystems.output.OutputConstants.OutputState;
import frc.robot.subsystems.output.OutputSubsystem;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveCommands;

@Logged
public class RobotContainer {
    public static final Swerve SWERVE = Swerve.getInstance();
    public static final OutputSubsystem outputSubsystem = new OutputSubsystem();
    public static final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

    private final CommandXboxController driverController = new CommandXboxController(OIConstants.kDriverControllerPort);

    public RobotContainer() {
        SWERVE.setDefaultCommand(SwerveCommands.getFieldRelativeOpenLoopSupplierDriveCommand(
                () -> -driverController.getLeftY()/4,
                () -> -driverController.getLeftX()/4,
                () -> -driverController.getRightX()/8
        ));
        // elevatorSubsystem.setDefaultCommand(ElevatorCommands.moveToHeight(ElevatorState.L1));
        configureButtonBindings();
        SmartDashboard.putData(elevatorSubsystem);
    }

    private void configureButtonBindings() {
        driverController.y().whileTrue(OutputCommands.output(OutputState.L1));
        driverController.x().whileTrue(OutputCommands.output(OutputState.L2L3));
        driverController.a().whileTrue(OutputCommands.output(OutputState.L4));
        driverController.b().whileTrue(OutputCommands.output(OutputState.STOP));
        
        driverController.povUp().onTrue(ElevatorCommands.moveToHeight(ElevatorState.L1));  
        driverController.povRight().onTrue(ElevatorCommands.moveToHeight(ElevatorState.L2)); 
        driverController.povDown().onTrue(ElevatorCommands.moveToHeight(ElevatorState.L3));
        driverController.povLeft().onTrue(ElevatorCommands.moveToHeight(ElevatorState.L4));

//        driverController.a().onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
//        driverController.b().onTrue(new InstantCommand(() -> swerveSubsystem.stopModules()));
    }
}
