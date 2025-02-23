package frc.robot.subsystems.output;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.output.OutputConstants.OutputState;

@Logged
public class Output extends SubsystemBase {
    private final DigitalInput beamBreaker = OutputConstants.beamBreaker;
    public boolean manualOverride = false;

    public void setMotorOutput(OutputState state) {
        if (!isCoralInside() || manualOverride) { 
            // Run motors if coral is NOT inside or manual override is active
            OutputConstants.up.set(state.upPower);
            OutputConstants.down.set(state.downPower);
        } else {
            stop(); // Stop if coral is inside
        }
    }

    public boolean isCoralInside() {
        return !beamBreaker.get(); // Assuming `false` means the beam is broken (coral is inside)
    }

    public void enableManualOverride() {
        manualOverride = true;
    }

    public void disableManualOverride() {
        manualOverride = false;
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

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Coral Inside", isCoralInside());
        SmartDashboard.putBoolean("Manual Override", manualOverride);
        SmartDashboard.putNumber("Up Motor Output", getUpOutput());
        SmartDashboard.putNumber("Down Motor Output", getDownOutput());
    }
}
