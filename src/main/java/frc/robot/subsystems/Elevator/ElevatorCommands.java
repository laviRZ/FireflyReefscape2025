package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Elevator.ElevatorConstants.ElevatorState;

public class ElevatorCommands {

    public static Command moveToHeight(ElevatorState state) {
        return new FunctionalCommand(
            () -> RobotContainer.ELEVATOR.initController(state), // Init: Set target height
            RobotContainer.ELEVATOR::runMotor, // Execute: Run the motor
            (interrupted) -> RobotContainer.ELEVATOR.stop(), // Stop motor if command ends
            () -> false, // Never auto-finish
            RobotContainer.ELEVATOR
        );
    }

    public static Command moveDown() {
        return new FunctionalCommand(
            () -> RobotContainer.ELEVATOR.initController(ElevatorState.L1), // Init: Set L1 height
            RobotContainer.ELEVATOR::runMotor, // Execute: Move down
            (interrupted) -> RobotContainer.ELEVATOR.stop(), // Stop when released
            () -> RobotContainer.ELEVATOR.getCurrentPosition() <= ElevatorState.L1.height, // Stop when at L1
            RobotContainer.ELEVATOR
        );
    }

    public static Command resetEncoder(Elevator elevator) {
        return new InstantCommand(elevator::resetEncoder, elevator);
    }
}
