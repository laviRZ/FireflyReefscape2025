package frc.robot.subsystems.Elevator;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;

public class ElevatorConstants {

    public static final int LIMIT_SWITCH_PORT = 1; 
    public static final DigitalInput limitSwitch = new DigitalInput(LIMIT_SWITCH_PORT);

    public enum ElevatorState {
        L4(1.2),
        L3(0.8),
        L2(0.5),
        L1(0.0);

        double height;

        ElevatorState(double height) {
            this.height = height;
        }
    }

    public static final int ELEVATOR_MOTOR_ID = 40;

    static final SparkMax elevatorMotor = new SparkMax(ELEVATOR_MOTOR_ID, MotorType.kBrushless);
    static final ProfiledPIDController controller
            = new ProfiledPIDController(
                    40,
                    0.0,
                    0,
                    new TrapezoidProfile.Constraints(3, 3)
            );
    static final ElevatorFeedforward FEEDFORWARD = new ElevatorFeedforward(0.37,0.69,5.6,0);

    // tune Feedforward

    // can change to 10:1 if needed
    public static double GEAR_RATIO = 10.0;
    public static final double SPOOL_DIAMETER_METERS = 0.1016;

    public static double ENCODER_TO_METERS
            = 1.26/53.453;
}
