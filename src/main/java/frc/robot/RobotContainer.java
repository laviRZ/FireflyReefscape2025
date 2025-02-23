package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Elevator.ElevatorCommands;
import frc.robot.subsystems.Elevator.ElevatorConstants.ElevatorState;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.output.OutputCommands;
import frc.robot.subsystems.output.OutputConstants.OutputState;
import frc.robot.subsystems.output.Output;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveCommands;

@Logged
public class RobotContainer {
    public static final Swerve SWERVE = Swerve.getInstance();
    public static final Output OUTPUT = new Output();
    public static final Elevator ELEVATOR = new Elevator();

    private final CommandXboxController driverController = new CommandXboxController(OIConstants.kDriverControllerPort);

    public RobotContainer() {
        SWERVE.setDefaultCommand(SwerveCommands.getFieldRelativeOpenLoopSupplierDriveCommand(
                () -> -driverController.getLeftY()/4,
                () -> -driverController.getLeftX()/4,
                () -> -driverController.getRightX()/8
        ));
        // elevatorSubsystem.setDefaultCommand(ElevatorCommands.moveToHeight(ElevatorState.L1));
        configureButtonBindings();
        SmartDashboard.putData(ELEVATOR);
    }

    private void configureButtonBindings() {
        driverController.a().and(driverController.povUp()).whileTrue(OutputCommands.output(OutputState.L1));
        driverController.a().and(driverController.povRight().or(driverController.povDown()) ).whileTrue(OutputCommands.output(OutputState.L2L3));
         driverController.a().and(driverController.povDown()).whileTrue(OutputCommands.output(OutputState.L2L3));
        driverController.a().and(driverController.povLeft()).whileTrue(OutputCommands.output(OutputState.L4));
        
        driverController.povUp().onTrue(ElevatorCommands.moveToHeight(ElevatorState.L1));  
        driverController.povRight().onTrue(ElevatorCommands.moveToHeight(ElevatorState.L2)); 
        driverController.povDown().onTrue(ElevatorCommands.moveToHeight(ElevatorState.L3));
        driverController.povLeft().onTrue(ElevatorCommands.moveToHeight(ElevatorState.L4));

//        driverController.a().onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
//        driverController.b().onTrue(new InstantCommand(() -> swerveSubsystem.stopModules()));
    }

    @Logged(name="Swerve")
    public Swerve logSwerve(){
        return SWERVE;
    }

    @Logged(name = "Output")
    public Output logOutput(){
        return OUTPUT;
    }

    @Logged(name = "Elevator")
    public Elevator logElevator(){
        return ELEVATOR;
    }

}
