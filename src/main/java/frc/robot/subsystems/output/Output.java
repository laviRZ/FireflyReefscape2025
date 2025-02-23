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

    public void stop() {
        OutputConstants.up.set(0);
        OutputConstants.down.set(0);
    }

    public double getUpOutput() {
        return OutputConstants.up.getAppliedOutput();
    }

    public double getDownOutput() {
        return OutputConstants.down.getAppliedOutput();
    }
    public boolean hasCoral(){
        return OutputConstants.beamBreaker.get();
    }

}
