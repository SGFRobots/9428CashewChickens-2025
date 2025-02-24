package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Algae extends SubsystemBase{
    private final TalonFX positionMotor;

    public Algae() {
        positionMotor = new TalonFX(Constants.MotorPorts.kAlgaePosMotorID);
    }

    public void setPower(double power) {
        positionMotor.set(power);
    }

    public void stop() {
        positionMotor.set(0);
    }
}
