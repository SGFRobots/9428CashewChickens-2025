package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {    
    // Motors
    private final SparkMax LeftMotor;
    private final SparkMax RightMotor;

    // Heights
    private double pos0;
    private double pos1;
    private double pos2;
    private double pos3;
    private int desiredLevel;
    private double desiredPosition;
    private double[] positionsList;
    private boolean override;

    public Elevator() {
        // Motors
        LeftMotor = new SparkMax(Constants.MotorPorts.kLElevator, MotorType.kBrushless);
        RightMotor = new SparkMax(Constants.MotorPorts.kRElevator, MotorType.kBrushless);

        // Reset Heights
        pos0 = getPosition();
        pos1 = pos0 + Constants.Mechanical.ElevatorLevelOneHeight;
        pos2 = pos0 + Constants.Mechanical.ElevatorLevelTwoHeight;
        pos3 = pos0 + Constants.Mechanical.ElevatorMaxHeight;
        desiredLevel = 0;
        desiredPosition = pos0;
        positionsList = new double[]{pos0, pos1, pos2, pos3};
        override = false;
    }

    // Enable elevator motors
    public void setPower(double power) {
        // Limit bottom and top height
        if (((getPosition() < pos3) && (power < 0)) || ((getPosition() > pos0) && (power > 0))) {
            stop();
        } else {
            // Set power to motors
            LeftMotor.set(power);
            RightMotor.set(power);
        }
    }

    // Toggle override mode
    public void setOverride(boolean override) {
        this.override = override;
    }

    // Get override mode
    public boolean getOverride() {
        return override;
    }

    // Get absolute current position
    public double getPosition() {
        return LeftMotor.getEncoder().getPosition();
    }
    
    // Get relative current position
    public double getPositionRelativeToZero() {
        return LeftMotor.getEncoder().getPosition() - pos0;
    }

    // Get the position the motors are trying to go to
    public double getDesiredPosition(){
        return desiredPosition;
    }

    public int getDesiredLevel() {
        return desiredLevel;
    }
    
    // Set reef level to go to
    public void setDesiredPosition(int level, SwerveSubsystem driveSubsystem){
        // Set position
        desiredPosition = positionsList[level];
        desiredLevel = level;
    }

    // Get position of motor based on reef level
    public double getPositionInList(int level){
        return positionsList[level];
    }

    // Set specific motor desired position
    public void setDesiredPosition(double height){
        desiredPosition = height;
    }

    // Stop motors
    public void stop() {
        LeftMotor.set(0);
        RightMotor.set(0);
    }

    // Reset all heights
    public void resetPositions() {
        pos0 = getPosition();
        pos1 = pos0 + Constants.Mechanical.ElevatorLevelOneHeight;
        pos2 = pos0 + Constants.Mechanical.ElevatorLevelTwoHeight;
        pos3 = pos0 + Constants.Mechanical.ElevatorMaxHeight;
        positionsList = new double[]{pos0, pos1, pos2, pos3};
    }

}
