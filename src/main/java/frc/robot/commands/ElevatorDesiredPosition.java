package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.controller.PIDController;


public class ElevatorDesiredPosition extends Command {
    private final Elevator mElevator;
    private final double level;
    private final PIDController levelPID;

    public ElevatorDesiredPosition(Elevator pElevator, double pLevel) {
        mElevator = pElevator;
        level = pLevel;
        levelPID = new PIDController(0.075, 0, 0);
    }

    @Override
    public void initialize() {}

    @Override 
    public void execute() {
        // double power = (mElevator.getPosition() < level) ? 0.35 : -0.35;
        double power = levelPID.calculate(mElevator.getPosition(), level);
        mElevator.setPower(power);
    }

    @Override
    public void end(boolean isFinished) {
        mElevator.stop();
    }

    @Override
    public boolean isFinished() {
        if (mElevator.getPosition() > level-1 && mElevator.getPosition() < level+1){
            return true;
        }
        return false;
    }
}
