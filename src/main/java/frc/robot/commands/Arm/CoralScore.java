package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Coral;

public class CoralScore extends Command {
    private final Coral mCoral;
    private final GenericHID mController;

    public CoralScore(Coral pCoral, GenericHID pController) {
        mCoral = pCoral;
        mController = pController;

        addRequirements(mCoral);
    }
    
    @Override
    public void initialize() {
    }

    @Override 
    public void execute() {
        // Checks if in driver control
        if (Robot.stage.equals("teleOp")) {
            // Sets motor power based on trigger pressed
            double triggerShootValue = mController.getRawAxis(Constants.Controllers.XBox.RightTriggerPort);
            double triggerIntakeValue = mController.getRawAxis(Constants.Controllers.XBox.LeftTriggerPort);
            if (Math.abs(triggerShootValue) > 0.05){ 
                mCoral.setPower(.5);
            } else if (Math.abs(triggerIntakeValue) > 0.05){ 
                mCoral.setPower(.05);
            } else{
                mCoral.stop();
            }
        }
    }

    @Override
    public void end(boolean isFinished) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
