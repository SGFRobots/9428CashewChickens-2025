package frc.robot.commands.Limelight;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;

public class LimeLightControl extends Command{
    private final Limelight mLimelight;

    public LimeLightControl(Limelight limelight) {
        mLimelight = limelight;
        addRequirements(mLimelight);
    }

    @Override
    public void initialize() {}

    @Override 
    public void execute() {
        mLimelight.displayData();
    }

    @Override
    public void end(boolean isFinished) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
