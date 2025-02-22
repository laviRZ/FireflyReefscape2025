package frc.robot.subsystems.Elevator;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Elevator.ElevatorConstants.ElevatorState;

@Logged
public class Elevator extends SubsystemBase {
    private final DigitalInput limitSwitch = ElevatorConstants.limitSwitch;

    public Elevator() {
        SparkMaxConfig motorconfig = new SparkMaxConfig();
        motorconfig.encoder.positionConversionFactor(ElevatorConstants.ENCODER_TO_METERS);
        motorconfig.encoder.velocityConversionFactor(ElevatorConstants.ENCODER_TO_METERS / 60);
        motorconfig.inverted(false);
        ElevatorConstants.elevatorMotor.configure(motorconfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        System.out.println("Skibidi toilet");

        if (isAtBottom()) {
            resetEncoder();
        }
    }

    public void initController(ElevatorState state) {
        ElevatorConstants.controller.setGoal(state.height);
        ElevatorConstants.controller.reset(getCurrentPosition(), getCurrentVelocity());
    }

    public void runMotor() {
        if (isAtBottom()) {
            stop();
            resetEncoder(); 
        } else {
            double feedback = ElevatorConstants.controller.calculate(getCurrentPosition());
            var setpoint = ElevatorConstants.controller.getSetpoint();
            double feedforward = ElevatorConstants.FEEDFORWARD.calculate(setpoint.velocity);
            ElevatorConstants.elevatorMotor.setVoltage(feedback + feedforward);
        }

        SmartDashboard.putNumber("target position", ElevatorConstants.controller.getSetpoint().position);
        SmartDashboard.putNumber("target velocity", ElevatorConstants.controller.getSetpoint().velocity);
        SmartDashboard.putNumber("current position", getCurrentPosition());
        SmartDashboard.putNumber("current velocity", getCurrentVelocity());
        SmartDashboard.putNumber("feedforward", ElevatorConstants.FEEDFORWARD.calculate(ElevatorConstants.controller.getSetpoint().velocity));
    }

    public void stop() {
        ElevatorConstants.elevatorMotor.set(0);
    }

    public double getCurrentPosition() {
        return ElevatorConstants.elevatorMotor.getEncoder().getPosition();
    }

    public double getCurrentVelocity() {
        return ElevatorConstants.elevatorMotor.getEncoder().getVelocity();
    }

    public boolean isAtBottom() {
        return limitSwitch.get(); 
    }

    public void resetEncoder() {
        ElevatorConstants.elevatorMotor.getEncoder().setPosition(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Elevator At Bottom", isAtBottom());
        SmartDashboard.putNumber("Elevator Position", getCurrentPosition());
    }
}
