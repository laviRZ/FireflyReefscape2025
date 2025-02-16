package frc.robot.subsystems.output;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.RobotContainer;

public class OutputCommands {
    public static Command outputL4() {
        return new RunCommand(
                () -> {
                    OutputConstants.up.set(-OutputConstants.kAnglePower);
                    OutputConstants.down.set(OutputConstants.kOutPower);
                },
                RobotContainer.outputSubsystem
        );
    }

    public static Command stop() {
        return new RunCommand(
                () -> {
                    OutputConstants.up.set(0);
                    OutputConstants.down.set(0);
                },
                RobotContainer.outputSubsystem
        );
    }
    public static Command outputStraight() {
        return new RunCommand(
                () -> {
                    OutputConstants.up.set(OutputConstants.kOutPower);
                    OutputConstants.down.set(OutputConstants.kOutPower);
                },
                RobotContainer.outputSubsystem

        );
    }
    public static Command outputL2L3() {
        return new RunCommand(
                () -> {
                    OutputConstants.up.set(OutputConstants.kAnglePower*1.5);
                    OutputConstants.down.set(OutputConstants.kOutPower);
                },
                RobotContainer.outputSubsystem
        );
    }
}
