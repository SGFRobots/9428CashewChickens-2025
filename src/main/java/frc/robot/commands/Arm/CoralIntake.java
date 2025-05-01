package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;

public class CoralIntake extends Command{
    private Coral mCoral;
    private Elevator mElevator;
    private boolean coralIn;
    private double power;
    
    public CoralIntake(Coral pCoral, Elevator pElevator) {
        // Initialize variables
        mCoral = pCoral;
        mElevator = pElevator;
    }
    
    @Override
    public void initialize() {
        power = 0.075;
        coralIn = false;
        mElevator.setDesiredPosition("coral", 0);
    }
    
    @Override 
    public void execute() {
        // Reverse the power when coral all the way in
        if (!mCoral.getInSensorBroken() && mCoral.getOutSensorBroken()){
            coralIn = true;
            power = -0.1;
        }
        // Apply power to the coral shooter
        mCoral.setPower(power);
    }

    @Override
    public void end(boolean isFinished) {
        // Stop the coral shooter
        mCoral.stop();
    }

    @Override
    public boolean isFinished() {
        // Stop the command when coral in and in sensor is broken
        if (coralIn && mCoral.getInSensorBroken() && mCoral.getOutSensorBroken()){
            return true;
        }
        return false;
    }
}
