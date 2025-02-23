package frc.robot.subsystems.output;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;

public class OutputConstants {
     public static final int BEAM_BREAKER_PORT = 2; 
    public static final DigitalInput beamBreaker = new DigitalInput(BEAM_BREAKER_PORT);
     public enum OutputState {
        L4 (-ANGLE_POWER, -OUT_POWER),
        L2L3 (-ANGLE_POWER,OUT_POWER),
        STOP (0, 0),
        L1 (-L1_TOP_MOTOR_POWER, -OUT_POWER);
        
        double upPower;
        double downPower;
        OutputState(double upPower, double downPower) {
            this.upPower = upPower;
            this.downPower = downPower;
        }
    }  



     static final SparkMax up = new SparkMax(30, SparkLowLevel.MotorType.kBrushless);
    static final SparkMax down = new SparkMax(31, SparkLowLevel.MotorType.kBrushless);
    static final double ANGLE_POWER = 0.08
     ,OUT_POWER= -0.3
     ,L1_TOP_MOTOR_POWER = 0.12;


}

