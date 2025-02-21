package frc.robot.subsystems.output;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.output.OutputConstants.OutputState;

public class OutputCommands {
    public static Command output(OutputState state) {
        return new RunCommand(
                () -> {
                   RobotContainer.outputSubsystem.setMotorOutput(state);
                },
                RobotContainer.outputSubsystem
        );
    }
}
