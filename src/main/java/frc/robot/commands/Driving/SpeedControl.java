package frc.robot.commands.Driving;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SwerveSubsystem;

public class SpeedControl extends Command{
    private final SwerveSubsystem mSubsystem;
    private final GenericHID mController;
    private final Elevator mElevator;

    public SpeedControl(SwerveSubsystem subsystem, GenericHID pController, Elevator pElevator) {
        // slow = new SlowMode(subsystem);
        // fast = new FastMode(subsystem);
        mController = pController;
        mSubsystem = subsystem;
        mElevator = pElevator;
    }

    @Override
    public void initialize() {}

    @Override 
    public void execute() {
        int level = mElevator.getDesiredLevel();

        // Normal mode when aligning
        if (mSubsystem.getFindingPos()) {
            mSubsystem.toggleFastMode(true);
            mSubsystem.toggleSlowMode(false);

        } else if (Robot.stage.equals("auto")) {
            // Fast mode always during auto
            mSubsystem.toggleFastMode(true);
            mSubsystem.toggleSlowMode(false);

        } else if (Robot.stage.equals("teleOp")) {
            if (level <= 2) {
                // Control speed based on controller inputs during teleOp
                boolean fast = (mController.getRawAxis(Constants.Controllers.selected.SwitchC) == -1);
                mSubsystem.toggleFastMode(fast);
                boolean slow =(mController.getRawAxis(Constants.Controllers.selected.SwitchC) == 1);
                mSubsystem.toggleSlowMode(slow);
                
            // Ensure speed is slow when the level is high
            } else if (level == 3) {
                mSubsystem.toggleFastMode(false);
                mSubsystem.toggleSlowMode(false);
            } else {
                mSubsystem.toggleSlowMode(true);
                mSubsystem.toggleFastMode(false);
            }
        }

        SmartDashboard.putBoolean("FastMode", mSubsystem.getFastMode());
        SmartDashboard.putBoolean("SlowMode", mSubsystem.getSlowMode());
        SmartDashboard.putBoolean("NormalMode", !mSubsystem.getSlowMode() && !mSubsystem.getFastMode());
    }

    @Override
    public void end(boolean isFinished) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
