package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.ProximitySensorMeasurementRate;
import com.revrobotics.ColorSensorV3.ProximitySensorResolution;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.I2C;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Coral extends SubsystemBase {
    private final SparkMax leftMotor;
    private final SparkMax rightMotor;
    private final ColorSensorV3 sensor;

    public Coral() {
        leftMotor = new SparkMax(Constants.MotorPorts.kLCoral, MotorType.kBrushless);
        rightMotor = new SparkMax(Constants.MotorPorts.kRCoral, MotorType.kBrushless);
        sensor = new ColorSensorV3(I2C.Port.kMXP);
        sensor.configureProximitySensor(ProximitySensorResolution.kProxRes11bit, ProximitySensorMeasurementRate.kProxRate100ms);
    }

    public void setPower(double power) {
        leftMotor.set(power);
        rightMotor.set(power);
    }

    public void stop() {
        leftMotor.set(0);
        rightMotor.set(0);
    }

    public double getSensorDist(){
        return sensor.getProximity();
    }
    
}
