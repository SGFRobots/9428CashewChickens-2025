package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    private final SparkMax LeftMotor;
    private final SparkMax RightMotor;
    private double lowestPos;
    private double highestPos;

    public Elevator() {
        LeftMotor = new SparkMax(Constants.MotorPorts.kLElevator, MotorType.kBrushless);
        RightMotor = new SparkMax(Constants.MotorPorts.kRElevator, MotorType.kBrushless);
        lowestPos = LeftMotor.getEncoder().getPosition();
        highestPos = lowestPos + Constants.Mechanical.ElevatorMaxHeight;
    }

    public void setPower(double power) {
        System.out.println(power);
        if (((getPosition() > highestPos) && (power > 0)) || ((getPosition() < lowestPos) && (power < 0))) {
            stop();
        } else {
            LeftMotor.set(power);
            RightMotor.set(power);
            SmartDashboard.putNumber("Elevator left position", LeftMotor.getEncoder().getPosition());
            SmartDashboard.putNumber("Elevator right position", RightMotor.getEncoder().getPosition());
            SmartDashboard.putNumber("Elevator position relative to 0", getPosition() - Constants.Mechanical.ElevatorLowestPosition);
        }
    }

    public double getPosition() {
        return LeftMotor.getEncoder().getPosition();
    }

    public void stop() {
        LeftMotor.set(0);
        RightMotor.set(0);
    }

    public void resetPositions() {
        lowestPos =getPosition();
        highestPos = lowestPos + Constants.Mechanical.ElevatorMaxHeight;
        System.out.println("reset: " + lowestPos + " and " + highestPos);
    }

}
