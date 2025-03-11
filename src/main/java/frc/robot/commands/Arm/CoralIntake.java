package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Coral;

public class CoralIntake extends Command{
    private Coral mCoral;
    
    public CoralIntake(Coral pCoral) {
        // Initialize variables
        mCoral = pCoral;
    }

    @Override
    public void initialize() {
    }
    
    @Override 
    public void execute() {
        // Apply power to the coral shooter
        mCoral.setPower(0.1);
    }

    @Override
    public void end(boolean isFinished) {
        // Stop the coral shooter
        mCoral.stop();
    }

    @Override
    public boolean isFinished() {
        // Stop the command when the out sensor is broken and the in sensor is not
        if (!mCoral.getInSensorBroken() && mCoral.getOutSensorBroken()){
            return true;
        }
        return false;
    }
}
