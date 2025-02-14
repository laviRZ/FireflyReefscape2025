package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType; 
import com.studica.frc.AHRS;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TESTOutputSubsystem extends SubsystemBase {

    AHRS gyro;
    
    SparkMax up = new SparkMax(5, MotorType.kBrushless);
    SparkMax low = new SparkMax(6, MotorType.kBrushless);
    static final boolean kInvertFollower = true;
    static final float kAnglePower = 0.08f;
    static final float kOutPower = -0.3f;
    
    public Command angle() {
        return run(() -> {
            up.set(-kAnglePower);
            low.set(kAnglePower);
        });
    }

    public Command stop() {
        return run(() -> {
            up.set(0);
            low.set(0);
        });
    }

    public Command outStaight() {
        return run(() -> {
            up.set(kOutPower);
            low.set(kOutPower);
        });
    }

    public Command outInAngle() {
        return run(() -> {
            // low.set(kOutPower);
            // up.set(kOutPower * 1.5);
            up.set(kOutPower);
            low.set(0.25 * kOutPower);
        });
    }

}
