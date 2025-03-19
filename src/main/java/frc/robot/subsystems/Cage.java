package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Cage extends SubsystemBase{
    private SparkMax mMotor;
    private double downPos;
    private double upPos;
    private double desiredPos;

    public Cage() {
        mMotor = new SparkMax(15, MotorType.kBrushless);
        resetPos();
    }

    public void setPower(double power) {
        mMotor.set(power);
    }

    public double getDesiredPos() {
        return desiredPos;
    }

    public double getDownPos() {
        return downPos;
    }

    public double getupPos() {
        return upPos;
    }

    public double getAbsolutePos() {
        return mMotor.getEncoder().getPosition();
    }

    public double getRelativePos() {
        return getAbsolutePos() - getDownPos();
    }

    public void setDesiredPos(int pos) {
        desiredPos = pos == 0 ? downPos : upPos;
    }

    public void resetPos() {
        downPos = mMotor.getEncoder().getPosition()-0.25;
        upPos = downPos + Constants.Mechanical.CageUpPos;
        desiredPos = downPos;
    }

    public SparkMax getMotor() {
        return mMotor;
    }
}
