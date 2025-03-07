package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Algae extends SubsystemBase{
    private final SparkMax positionMotor;
    private final SparkMax wheelMotor;

    public Algae() {
        // Set up motors
        positionMotor = new SparkMax(Constants.MotorPorts.kAlgaePosMotorID, MotorType.kBrushless);
        wheelMotor = new SparkMax(Constants.MotorPorts.kAlgaeWheelMotor, MotorType.kBrushless);
    }

    public void setPosPower(double power) {
        // Apply power to position motor
        positionMotor.set(power);
    }

    public void setWheelPower(double power) {
        // Sets algae wheel motor power
        wheelMotor.set(power);
    }

    public void stop() {
        // Stops motors
        positionMotor.set(0);
        wheelMotor.set(0);
    }

    public double getPosition() {
        // Gets position motor 
        return positionMotor.getEncoder().getPosition();
    }

    public void telemetry() {
        
    }
}
