package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.math.controller.PIDController;


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
        mElevator.setPower(power);
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
