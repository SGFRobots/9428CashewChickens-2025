package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

import java.lang.reflect.Member;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ElevatorDesiredPosition extends Command {
    private final Elevator mElevator;
    private final PIDController levelPID;

    public ElevatorDesiredPosition(Elevator pElevator) {
        // Initialize variables 
        mElevator = pElevator;
        levelPID = new PIDController(0.0035, 0, 0);
        addRequirements(pElevator);
    }

    @Override
    public void initialize() {}

    @Override 
    public void execute() {
        // Set power to elevator based on the error betweeen current position and desired position
        double power = levelPID.calculate(mElevator.getPosition(), mElevator.getDesiredPosition());
        // Decrease power if moving down to prevent slamming
        if (power > 0) {
            power *= 0.75;
        }
        mElevator.setPower(power);

        SmartDashboard.putNumber("elevatorpowet", power);
        SmartDashboard.putNumber("elevator level", mElevator.getDesiredLevel());
        SmartDashboard.putNumber("el", mElevator.getArraySize());
    }

    @Override
    public void end(boolean isFinished) {
        // Stop motors
        mElevator.stop();
    }

    @Override
    public boolean isFinished() {
        // Joystick override
        return mElevator.getOverride();
    }
}
