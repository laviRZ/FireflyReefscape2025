package frc.robot.subsystems.output;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.output.OutputConstants.OutputState;

public class OutputCommands {

    // Regular output command (runs motor based on state)
    public static Command output(OutputState state) {
        return new RunCommand(
                () -> {
                    if (!RobotContainer.OUTPUT.isCoralInside() || RobotContainer.OUTPUT.manualOverride) {
                        RobotContainer.OUTPUT.setMotorOutput(state);
                    } else {
                        RobotContainer.OUTPUT.stop();
                    }
                },
                RobotContainer.OUTPUT
        );
    }
}
