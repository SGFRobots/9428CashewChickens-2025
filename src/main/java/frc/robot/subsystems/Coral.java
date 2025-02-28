package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.ProximitySensorMeasurementRate;
import com.revrobotics.ColorSensorV3.ProximitySensorResolution;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DigitalInput;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Coral extends SubsystemBase {
    private final SparkMax leftMotor;
    private final SparkMax rightMotor;
    private final ColorSensorV3 inSensor;
    private final ColorSensorV3 outSensor;
    private final DigitalInput inBeamBreakSensor;
    private final DigitalInput outBeamBreakSensor;
    public boolean coralIn;

    public Coral() {
        leftMotor = new SparkMax(Constants.MotorPorts.kLCoral, MotorType.kBrushless);
        rightMotor = new SparkMax(Constants.MotorPorts.kRCoral, MotorType.kBrushless);
        inSensor = new ColorSensorV3(I2C.Port.kMXP);
        inSensor.configureProximitySensor(ProximitySensorResolution.kProxRes11bit, ProximitySensorMeasurementRate.kProxRate100ms);
        outSensor = new ColorSensorV3(I2C.Port.kMXP);
        outSensor.configureProximitySensor(ProximitySensorResolution.kProxRes11bit, ProximitySensorMeasurementRate.kProxRate100ms);
        inBeamBreakSensor = new DigitalInput(3);
        outBeamBreakSensor = new DigitalInput(2);
        coralIn = false;
    }

    public void setPower(double power) {
        leftMotor.set(power);
        rightMotor.set(power);
    }

    public void stop() {
        leftMotor.set(0);
        rightMotor.set(0);
    }

    public boolean getInSensorBroken(){
        return !inBeamBreakSensor.get();
    }
    
    public boolean getOutSensorBroken(){
        return !outBeamBreakSensor.get();
    }

    public void updateCoralStatus() {
        if (getInSensorBroken() || getOutSensorBroken()){
            coralIn = true;
        }
    }
}
