package frc.robot.commands.Arm;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        // mCoral.setPower(.5);
        System.out.println("Coral Sensor Broken " + mCoral.getInSensorBroken());
        // if (mCoral.getInSensorBroken()){
        //     mCoral.coralIn = true;
        // }
    }

    @Override
    public void end(boolean isFinished) {
        mCoral.stop();
    }

    @Override
    public boolean isFinished() {
        if (mCoral.coralIn && !mCoral.getInSensorBroken()){
            return true;
        }
        return false;
    }
}
