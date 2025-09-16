package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Coral;

// Wait until coral is inside intake
public class WaitForCoral extends Command{
    private final Coral mCoral;
    public WaitForCoral(Coral pCoral) {
        mCoral = pCoral;
    }

    @Override
    public void initialize() {}

    @Override 
    public void execute() {}

    @Override
    public void end(boolean isFinished) {}

    @Override
    public boolean isFinished() {
        return mCoral.getInSensorBroken();
    }
}
