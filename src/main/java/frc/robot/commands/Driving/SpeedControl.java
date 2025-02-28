package frc.robot.commands.Driving;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class SpeedControl extends Command{
    private final SwerveSubsystem mSubsystem;
    private final GenericHID mController;

    public SpeedControl(SwerveSubsystem subsystem, GenericHID pController) {
        // slow = new SlowMode(subsystem);
        // fast = new FastMode(subsystem);
        mController = pController;
        mSubsystem = subsystem;
    }

    @Override
    public void initialize() {}

    @Override 
    public void execute() {
        boolean fast = mController.getRawAxis(Constants.Controllers.selected.SwitchC) == 1;
        mSubsystem.toggleFastMode(fast);
        boolean slow = mController.getRawAxis(Constants.Controllers.selected.SwitchC) == -1;
        mSubsystem.toggleSlowMode(slow);
    }

    @Override
    public void end(boolean isFinished) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
