package frc.robot.subsystems.output;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

public class OutputConstants {
     static final SparkMax up = new SparkMax(5, SparkLowLevel.MotorType.kBrushless);
    static final SparkMax down = new SparkMax(6, SparkLowLevel.MotorType.kBrushless);
    static final boolean kInvertFollower = true;
    static final float kAnglePower = 0.08f;
    static final float kOutPower = -0.3f;

}
