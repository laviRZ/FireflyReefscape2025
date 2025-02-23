package frc.robot.subsystems.output;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.output.OutputConstants.OutputState;

@Logged
public class Output extends SubsystemBase {

    public void setMotorOutput(OutputState state) {
        OutputConstants.up.set(state.upPower);
        OutputConstants.down.set(state.downPower);
    }

    public double getUpOutput(){
        return OutputConstants.up.getAppliedOutput();
    }

    public double getDownOutput(){
        return OutputConstants.down.getAppliedOutput();
    }
}
