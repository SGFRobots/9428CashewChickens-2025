package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
        positionMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void setPosPower(double power) {
        positionMotor.set(power);
    }

    public void setWheelPower(double power) {
        wheelMotor.set(power);
    }

    public void stop() {
        positionMotor.set(0);
        wheelMotor.set(0);
    }

    public double getPosition() {
        return positionMotor.getPosition().getValueAsDouble();
    }

    public void telemetry() {
        SmartDashboard.putNumber("Wheel algae", wheelMotor.getDeviceId());
    }
}
