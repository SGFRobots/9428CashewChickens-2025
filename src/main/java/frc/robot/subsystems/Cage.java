package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Cage extends SubsystemBase{
    private SparkMax mPullieMotor;
    private SparkMax mLiftyMotor;
    private double pullieDownPos;
    private double pullieUpPos;
    private double pullieDesiredPos;
    private int pullieDesiredIndex;
    private double liftyUpPos;
    private double liftyDownPos;

    public Cage() {
        mPullieMotor = new SparkMax(15, MotorType.kBrushless);
        mLiftyMotor = new SparkMax(16, MotorType.kBrushless);
        resetPos();
    }

    // Set power to pullie motor
    public void setPulliePower(double power) {
        mPullieMotor.set(power);
    }

    // Set power to lifty power (only when trying to go up)
    public void setLiftyPower(double power) {
        power = pullieDesiredIndex == 0 ? 0 : power;
        System.out.println(power);
        mLiftyMotor.set(power);
    }

    // get current target position of pullie motor
    public double getPullieDesiredPos() {
        return pullieDesiredPos;
    }

    // get the up position of lifty motor
    public double getLiftyUpPos() {
        return liftyUpPos;
    }

    // get the down position of lifty motor
    public double getLiftyDownPos() {
        return liftyDownPos;
    }

    // get the current index of position the cage is trying to go to (0 = down; 1 = up)
    public int getPullieDesiredIndex() {
        return pullieDesiredIndex;
    }

    // get the down position of pullie motor
    public double getPullieDownPos() {
        return pullieDownPos;
    }

    // get up position of pullie motor
    public double getPullieUpPos() {
        return pullieUpPos;
    }

    // get current encoder position of pullie motor
    public double getPullieAbsolutePos() {
        return mPullieMotor.getEncoder().getPosition();
    }

    // get current encoder position of lifty motor
    public double getLiftyAbsolutePos() {
        return mLiftyMotor.getEncoder().getPosition();
    }

    // set the desired position of the pullie motor
    public void setDesiredPos(int pos) {
        pullieDesiredIndex = pos;
        if (pos == 0) {
            pullieDesiredPos = pullieDownPos;
        } else {
            pullieDesiredPos = pullieUpPos;
        }
    }

    // reset positions of both motors
    public void resetPos() {
        pullieDownPos = getPullieAbsolutePos()-0.25;
        pullieUpPos = pullieDownPos + Constants.Mechanical.CagePullieUpPos;
        pullieDesiredPos = pullieDownPos;
        pullieDesiredIndex = 0;
        liftyDownPos = getLiftyAbsolutePos();
        liftyUpPos = liftyDownPos + Constants.Mechanical.CageLiftyUpPos;
    }
}
