package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;
import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ElevatorDesiredPosition extends Command {
    private final Elevator mElevator;
    private final PIDController levelPID;
    private final Coral mCoral;

    public ElevatorDesiredPosition(Elevator pElevator, Coral pCoral) {
        mElevator = pElevator;
        levelPID = new PIDController(0.0035, 0, 0);
        mCoral = pCoral;
    }

    @Override
    public void initialize() {}

    @Override 
    public void execute() {
            double power = levelPID.calculate(mElevator.getPosition(), mElevator.getDesiredPosition());
            mElevator.setPower(power);
            SmartDashboard.putNumber("desired pos elev", mElevator.getDesiredPosition());
        // if (power < 0.05) {
        //     mCoral.setPower(0.5);
        // }
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
