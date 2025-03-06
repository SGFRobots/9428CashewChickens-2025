package frc.robot.commands.Arm;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Coral;


public class CoralOuttake extends Command{
    private final Coral mCoral;

    public CoralOuttake(Coral pCoral) {
        mCoral = pCoral;
    }

    @Override
    public void initialize() {
        mCoral.coralIn = true;
    }

    @Override 
    public void execute() {
        System.out.println("Shooting");
        mCoral.setPower(.5);
    }
    
    @Override
    public void end(boolean isFinished) {
        mCoral.stop();
    }
    
    @Override
    public boolean isFinished() {
        // if (!mCoral.getOutSensorBroken()){
        //     mCoral.coralIn = false;
        //     return true;
        // }
        return false;
    }
}
