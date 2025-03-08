package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Coral;
import frc.robot.commands.Arm.CoralIntake;
import frc.robot.subsystems.Algae;

public class CoralScore extends Command {
    private final Coral mCoral;
    private final GenericHID mController;
    private final CoralIntake mCoralIntake;
    private final Algae mAlgae;

    public CoralScore(Coral pCoral, Algae pAlgae, GenericHID pController) {
        mCoral = pCoral;
        mController = pController;
        mAlgae = pAlgae;
        mCoralIntake = new CoralIntake(mCoral, mAlgae);
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
                mCoralIntake.schedule();
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
