package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Coral;
import frc.robot.commands.Arm.CoralIntake;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Elevator;

public class CoralScore extends Command {
    private final Coral mCoral;
    private final GenericHID mController;
    private final CoralIntake mCoralIntake;
    private final Algae mAlgae;
    private final Elevator mElevator;

    public CoralScore(Coral pCoral, Algae pAlgae, Elevator pElevator, GenericHID pController) {
        mCoral = pCoral;
        mController = pController;
        mAlgae = pAlgae;
        mElevator = pElevator;
        mCoralIntake = new CoralIntake(mCoral);
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
                // Shoot coral sideways if scoring on the lowest level
                if (mElevator.getDesiredLevel() == 1){
                    mCoral.setIndividualPower(0.01, -0.5);
                }
                // Otherwise, shoot straight
                else{
                    mCoral.setPower(.5);
                }
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
