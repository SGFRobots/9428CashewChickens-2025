package frc.robot.commands.Arm;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Coral;


public class CoralIntake extends Command{
    private final Coral mCoral;

    public CoralIntake(Coral pCoral) {
        mCoral = pCoral;
    }

    @Override
    public void initialize() {
        mCoral.coralIn = false;
    }

    @Override 
    public void execute() {
        mCoral.setPower(.5);
        if (mCoral.getInSensorDist() < 500){
            mCoral.coralIn = true;
        }
    }

    @Override
    public void end(boolean isFinished) {
        mCoral.stop();
    }

    @Override
    public boolean isFinished() {
        if (mCoral.coralIn && mCoral.getInSensorDist() > 500){
            return true;
        }
        return false;
    }
}
