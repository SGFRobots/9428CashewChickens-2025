package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Algae extends SubsystemBase{
    private final SparkMax positionMotor;
    private final SparkMax wheelMotor;
    private double originalPos;
    private double upPos;
    private double desiredPos;

    public Algae() {
        // Set up motors
        positionMotor = new SparkMax(Constants.MotorPorts.kAlgaePosMotorID, MotorType.kBrushless);
        wheelMotor = new SparkMax(Constants.MotorPorts.kAlgaeWheelMotor, MotorType.kBrushless);
        originalPos = getAbsolutePos();
        upPos = originalPos + Constants.Mechanical.AlgaeUpPos;
        desiredPos = originalPos;
    }
    
    public void setPosPower(double power) {
        // Apply power to position motor
        positionMotor.set(power);
    }

    public void setDesiredPos(int pos) {
        if (pos == 0) {
            desiredPos = originalPos;
        } else {
            desiredPos = upPos;
        }
    }

    public double getDesiredPos() {
        return desiredPos;
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

    public double getRelativePos() {
        // Gets position motor 
        return positionMotor.getEncoder().getPosition() - getZeroPos();
    }

    public double getAbsolutePos() {
        return positionMotor.getEncoder().getPosition();
    }

    public double getZeroPos() {
        return originalPos;
    }

    public void telemetry() {
        
    }

    public void resetPos() {
        originalPos = getAbsolutePos();
        upPos = originalPos + Constants.Mechanical.AlgaeUpPos;
        desiredPos = originalPos;
    }
}
