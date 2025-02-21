package frc.robot.subsystems.Elevator;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Elevator.ElevatorConstants.ElevatorState;


public class ElevatorSubsystem extends SubsystemBase {
    public ElevatorSubsystem() {
        SparkMaxConfig motorconfig = new SparkMaxConfig();
        motorconfig.encoder.positionConversionFactor(ElevatorConstants.ENCODER_TO_METERS);
        motorconfig.encoder.velocityConversionFactor(ElevatorConstants.ENCODER_TO_METERS/60);
        motorconfig.inverted(true);
        ElevatorConstants.elevatorMotor.configure(motorconfig, ResetMode.kNoResetSafeParameters,PersistMode.kPersistParameters);
        System.out.println("Skibidi toilet");

    }

    public void initController(ElevatorState state) {
       
        ElevatorConstants.controller.setGoal(state.height);
        ElevatorConstants.controller.reset(getCurrentPosition(),getCurrentVelocity());
    }
    public void runMotor(){
        double feedback =ElevatorConstants.controller.calculate(getCurrentPosition());
        var setpoint = ElevatorConstants.controller.getSetpoint();
        double feedforward =ElevatorConstants.FEEDFORWARD.calculate(setpoint.velocity);
        ElevatorConstants.elevatorMotor.setVoltage(feedback + feedforward);
        SmartDashboard.putNumber("target position", setpoint.position);
        SmartDashboard.putNumber("target velocity", setpoint.velocity);
        SmartDashboard.putNumber("currect position", getCurrentPosition());
        SmartDashboard.putNumber("currect velocity",getCurrentVelocity());
        SmartDashboard.putNumber("feedfoward", feedforward);
    
    }
    public void stop(){
        ElevatorConstants.elevatorMotor.set(0);
    }

    public double getCurrentPosition() {
        return ElevatorConstants.elevatorMotor.getEncoder().getPosition();
    }
    public double getCurrentVelocity() {
        return ElevatorConstants.elevatorMotor.getEncoder().getVelocity();
    }

    public void resetEncoder() {
        ElevatorConstants.elevatorMotor.getEncoder().setPosition(0);
    }
}
    

