package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class ElevatorCommands {
    public static Command moveToHeight(ElevatorConstants.ElevatorState state) {
      return new FunctionalCommand(()->{
        RobotContainer.elevatorSubsystem.initController(state);
      }
        ,RobotContainer.elevatorSubsystem::runMotor,
         (interrupted)-> {
            RobotContainer.elevatorSubsystem.stop();
         }
         ,()->false,
         RobotContainer.elevatorSubsystem
         );
    }

    public static Command resetEncoder(ElevatorSubsystem elevator) {
        return new InstantCommand(elevator::resetEncoder, elevator);
    }
    
}
