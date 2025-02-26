package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.controller.PIDController;


public class ElevatorDesiredPosition extends Command {
    private final Elevator mElevator;
    private final double level;
    private final PIDController levelPID;

    public ElevatorDesiredPosition(Elevator pElevator, int pLevel) {
        mElevator = pElevator;
        // level = pLevel;
        level = mElevator.getDesiredPosition(pLevel);
        levelPID = new PIDController(0.0035, 0, 0);
    }

    @Override
    public void initialize() {}

    @Override 
    public void execute() {
        // double power = (mElevator.getPosition() < level) ? 0.35 : -0.35;
        double power = levelPID.calculate(mElevator.getPosition(), level);
        // double power = -0.5;
        mElevator.setPower(power);
    }

    @Override
    public void end(boolean isFinished) {
        mElevator.stop();
    }

    @Override
    public boolean isFinished() {
        return mElevator.getOverride();
    }
}
