package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    private final SparkMax LeftMotor;
    private final SparkMax RightMotor;

    public Elevator() {
        LeftMotor = new SparkMax(Constants.MotorPorts.kLElevator, MotorType.kBrushless);
        RightMotor = new SparkMax(Constants.MotorPorts.kRElevator, MotorType.kBrushless);
    }

    public void setPower(double power) {
        LeftMotor.set(power);
        RightMotor.set(power);
    }

    public void stop() {
        LeftMotor.set(0);
        RightMotor.set(0);
    }
}
