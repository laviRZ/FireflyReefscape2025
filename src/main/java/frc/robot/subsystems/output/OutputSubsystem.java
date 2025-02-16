package frc.robot.subsystems.output;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.studica.frc.AHRS;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OutputSubsystem extends SubsystemBase {

    AHRS gyro;

    SparkMax up = new SparkMax(5, MotorType.kBrushless);
    SparkMax down = new SparkMax(6, MotorType.kBrushless);
    static final boolean kInvertFollower = true;
    static final float kAnglePower = 0.08f;
    static final float kOutPower = -0.3f;

    public Command outputL4() {
        return new RunCommand(
                () -> {
                    up.set(-kAnglePower);
                    down.set(kOutPower);
                },
                this
        );
    }

    public Command stop() {
        return new RunCommand(
                () -> {
                    up.set(0);
                    down.set(0);
                },
                this
        );
    }
    public Command outputStraight() {
        return new RunCommand(
                () -> {
                    up.set(kOutPower);
                    down.set(kOutPower);
                },
                this
        );
    }
    public Command outputL2L3() {
        return new RunCommand(
                () -> {
                    up.set(kOutPower*1.5);
                    down.set( kOutPower);
                },
                this
        );
    }




}
