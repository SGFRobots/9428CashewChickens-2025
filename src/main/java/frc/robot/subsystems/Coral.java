package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Coral extends SubsystemBase {
    private final SparkMax leftMotor;
    private final SparkMax rightMotor;
    private DigitalInput inBeamBreakSensor;
    private DigitalInput outBeamBreakSensor;
    public boolean coralIn;
    private final Spark mLed;

    public Coral() {
        // Initialize variables
        leftMotor = new SparkMax(Constants.MotorPorts.kLCoral, MotorType.kBrushless);
        rightMotor = new SparkMax(Constants.MotorPorts.kRCoral, MotorType.kBrushless);
        inBeamBreakSensor = new DigitalInput(2);
        outBeamBreakSensor = new DigitalInput(1);
        coralIn = false;
        mLed = new Spark(Constants.Mechanical.LEDChannel);
    }

    public void setPower(double power) {
        // Sets power to motors
        leftMotor.set(-power);
        rightMotor.set(-power*0.45);
        mLed.set(0.5);
    }

    public void setIndividualPower(double rightPower, double leftPower){
        leftMotor.set(leftPower);
        rightMotor.set(rightPower);
    }

    public void stop() {
        // Stops all coral wheel motors
        leftMotor.set(0);
        rightMotor.set(0);
    }

    public boolean getInSensorBroken(){
        // Checks the interior sensor
        return !inBeamBreakSensor.get();
    }
    
    public boolean getOutSensorBroken(){
        // Checks the exterior sensor
        return !outBeamBreakSensor.get();
    }

    public void updateCoralStatus() {
        // Intakes coral based on sensors 
        if (getInSensorBroken() || getOutSensorBroken()){
            coralIn = true;
        }
    }
}
