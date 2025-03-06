package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Coral;

public class CoralScore extends Command {
    private final Coral mCoral;
    private final GenericHID mController;
    private boolean startWithCoral;
    private double power;

    public CoralScore(Coral pCoral, GenericHID pController) {
        mCoral = pCoral;
        mController = pController;
        power = 1.5;

        addRequirements(mCoral);
    }
    
    @Override
    public void initialize() {
        // startWithCoral = mCoral.coralIn;
        // power = startWithCoral ? -0.5 : 0.5;
    }

    @Override 
    public void execute() {
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

    @Override
    public void end(boolean isFinished) {}

    @Override
    public boolean isFinished() {
        // return startWithCoral != mCoral.coralIn;
        return false;
    }
}
