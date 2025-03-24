package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Cage extends SubsystemBase{
    private SparkMax mMotor;
    private double downPos;
    private double hangPos;
    private double upPos;
    private double desiredPos;
    private int desiredIndex;

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

    public int getDesiredIndex() {
        return desiredIndex;
    }

    public double getDownPos() {
        return downPos;
    }

    public double getupPos() {
        return upPos;
    }

    public double getHangPos() {
        return hangPos;
    }

    public double getAbsolutePos() {
        return mMotor.getEncoder().getPosition();
    }

    public double getRelativePos() {
        return getAbsolutePos() - getDownPos();
    }

    public void setDesiredPos(int pos) {
        desiredIndex = pos;
        if (pos == 0) {
            desiredPos = downPos;
        } else if (pos == 1) {
            desiredPos = hangPos;
        } else {
            desiredPos = upPos;
        }
    }

    public void resetPos() {
        downPos = mMotor.getEncoder().getPosition()-0.25;
        hangPos = downPos + Constants.Mechanical.CageHangPos;
        upPos = downPos + Constants.Mechanical.CageUpPos;
        desiredPos = downPos;
        desiredIndex = 0;
    }

    public SparkMax getMotor() {
        return mMotor;
    }
}
