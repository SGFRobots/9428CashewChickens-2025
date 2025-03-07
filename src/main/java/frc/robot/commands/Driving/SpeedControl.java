package frc.robot.commands.Driving;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
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
        // Control the speed with controller if the level is low
        if (level <= 1) {
            boolean fast = (mController.getRawAxis(Constants.Controllers.selected.SwitchC) == 1);
            mSubsystem.toggleFastMode(fast);
            boolean slow =(mController.getRawAxis(Constants.Controllers.selected.SwitchC) == -1);
            mSubsystem.toggleSlowMode(slow);
        // Ensure speed is slow when the level is high
        } else {
            mSubsystem.toggleSlowMode(true);
        }
    }

    @Override
    public void end(boolean isFinished) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
