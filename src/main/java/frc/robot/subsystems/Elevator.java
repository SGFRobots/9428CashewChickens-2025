package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    private final SparkMax LeftMotor;
    private final SparkMax RightMotor;
    private double pos0;
    private double pos1;
    private double pos2;
    private double pos3;
    private double desiredPosition;
    private double[] positionsList;
    private boolean override;

    public Elevator() {
        LeftMotor = new SparkMax(Constants.MotorPorts.kLElevator, MotorType.kBrushless);
        RightMotor = new SparkMax(Constants.MotorPorts.kRElevator, MotorType.kBrushless);
        pos0 = getPosition();
        pos1 = pos0 + Constants.Mechanical.ElevatorLevelOneHeight;
        pos2 = pos0 + Constants.Mechanical.ElevatorLevelTwoHeight;
        pos3 = pos0 + Constants.Mechanical.ElevatorMaxHeight;
        desiredPosition = pos0;
        positionsList = new double[]{pos0, pos1, pos2, pos3};
        override = false;
    }

    public void setPower(double power) {
        // System.out.println(power);
        if (((getPosition() < pos3) && (power < 0)) || ((getPosition() > pos0) && (power > 0))) {
            stop();
        } else {
            LeftMotor.set(power);
            RightMotor.set(power);
            SmartDashboard.putNumber("Elevator power", power);
            SmartDashboard.putNumber("Elevator left position", LeftMotor.getEncoder().getPosition());
            SmartDashboard.putNumber("Elevator right position", RightMotor.getEncoder().getPosition());
            SmartDashboard.putNumber("Elevator position relative to 0", getPositionRelativeToZero());
            SmartDashboard.putNumber("Elevator Position 3", pos3);
        }
    }

    public void setOverride(boolean override) {
        this.override = override;
    }

    public boolean getOverride() {
        return override;
    }

    public double getPosition() {
        return LeftMotor.getEncoder().getPosition();
    }
    
    public double getPositionRelativeToZero() {
        return LeftMotor.getEncoder().getPosition() - pos0;
    }

    public double getDesiredPosition(){
        return desiredPosition;
    }
    
    public void setDesiredPosition(int level, SwerveSubsystem driveSubsystem){
        desiredPosition = positionsList[level];
        boolean fast = level <= 1;
        driveSubsystem.toggleFastMode(fast);
        boolean slow = level == 3;
        driveSubsystem.toggleSlowMode(slow);
    }
    public double getPositionInList(int level){
        return positionsList[level];
    }

    public void setDesiredPosition(double height){
        desiredPosition = height;
    }

    public void stop() {
        LeftMotor.set(0);
        RightMotor.set(0);
    }

    public void resetPositions() {
        pos0 = getPosition();
        pos1 = pos0 + Constants.Mechanical.ElevatorLevelOneHeight;
        pos2 = pos0 + Constants.Mechanical.ElevatorLevelTwoHeight;
        pos3 = pos0 + Constants.Mechanical.ElevatorMaxHeight;
        positionsList = new double[]{pos0, pos1, pos2, pos3};
        // System.out.println("reset: " + pos0 + " and " + pos3);
    }

}
