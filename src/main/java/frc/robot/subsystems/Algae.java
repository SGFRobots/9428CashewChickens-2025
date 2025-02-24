package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Algae extends SubsystemBase{
    private final TalonFX positionMotor;
    private final SparkMax wheelMotor;

    public Algae() {
        positionMotor = new TalonFX(Constants.MotorPorts.kAlgaePosMotorID);
        wheelMotor = new SparkMax(Constants.MotorPorts.kAlgaeWheelMotor, MotorType.kBrushless);
    }

    public void setPosPower(double power) {
        positionMotor.set(power);
    }

    public void setWheelPower(double power) {
        wheelMotor.set(power);
    }

    public void stop() {
        positionMotor.set(0);
    }

    public void telemetry() {
        SmartDashboard.putNumber("Wheel algae", wheelMotor.getDeviceId());
    }
}
