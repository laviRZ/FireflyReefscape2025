package frc.robot.subsystems.output;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.studica.frc.AHRS;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.output.OutputConstants.OutputState;

public class OutputSubsystem extends SubsystemBase {

    public void setMotorOutput(OutputState state) {
        OutputConstants.up.set(state.upPower);
        OutputConstants.down.set(state.downPower);
    }
}
