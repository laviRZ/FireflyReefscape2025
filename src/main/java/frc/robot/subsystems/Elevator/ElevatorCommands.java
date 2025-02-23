package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class ElevatorCommands {
    public static Command moveToHeight(ElevatorConstants.ElevatorState state) {
      return new FunctionalCommand(()->{
        RobotContainer.ELEVATOR.initController(state);
      }
        ,RobotContainer.ELEVATOR::runMotor,
         (interrupted)-> {
            RobotContainer.ELEVATOR.stop();
         }
         ,()->false,
         RobotContainer.ELEVATOR
         );
    }

    public static Command resetEncoder(Elevator elevator) {
        return new InstantCommand(elevator::resetEncoder, elevator);
    }
    
}
