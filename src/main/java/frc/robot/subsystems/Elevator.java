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
    private double coralPos1;
    private double coralPos2;
    private double coralPos3;
    private double algaePos1;
    private double algaePos2;
    private int desiredLevel;
    private double desiredPosition;
    private double[] coralPositionsList;
    private double[] algaePositionsList;
    private boolean override;

    public Elevator() {
        // Motors
        LeftMotor = new SparkMax(Constants.MotorPorts.kLElevator, MotorType.kBrushless);
        RightMotor = new SparkMax(Constants.MotorPorts.kRElevator, MotorType.kBrushless);

        // Reset Heights
        pos0 = getPosition();
        coralPos1 = pos0 + Constants.Mechanical.ElevatorLevelOneHeight;
        coralPos2 = pos0 + Constants.Mechanical.ElevatorLevelTwoHeight;
        coralPos3 = pos0 + Constants.Mechanical.ElevatorMaxHeight;
        algaePos1 = pos0 + Constants.Mechanical.ElevatorAlgaeOneHeight;
        algaePos2 = pos0 + Constants.Mechanical.ElevatorAlgaeTwoHeight;
        desiredLevel = 0;
        desiredPosition = pos0;
        coralPositionsList = new double[]{pos0, coralPos1, coralPos2, coralPos3};
        algaePositionsList = new double[]{algaePos1, algaePos2};
        override = false;
    }

    // Enable elevator motors
    public void setPower(double power) {
        // Limit bottom and top height
        if (((getPosition() < coralPos3) && (power < 0)) || ((getPosition() > pos0) && (power > 0))) {
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
    public void setDesiredPosition(String gamePiece, int level, SwerveSubsystem driveSubsystem){
        // Set position
        if (gamePiece.equals("coral")){
            desiredPosition = coralPositionsList[level];
        }
        else{
            desiredPosition = algaePositionsList[level];
        }
        desiredLevel = level;
    }

    // Get position of motor based on reef level
    public double getPositionInList(int level){
        return coralPositionsList[level];
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
        coralPos1 = pos0 + Constants.Mechanical.ElevatorLevelOneHeight;
        coralPos2 = pos0 + Constants.Mechanical.ElevatorLevelTwoHeight;
        coralPos3 = pos0 + Constants.Mechanical.ElevatorMaxHeight;
        coralPositionsList = new double[]{pos0, coralPos1, coralPos2, coralPos3};
    }

}
