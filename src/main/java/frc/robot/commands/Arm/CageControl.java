package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Cage;

public class CageControl extends Command {
    private final Cage mCage;
    private final double pulliePosDeadzone;
    private final double liftyPosDeadzone;

    public CageControl(Cage pCage) {
        mCage = pCage;
        pulliePosDeadzone = 3;
        liftyPosDeadzone = 0.005;
        addRequirements(mCage);
    }

    @Override
    public void initialize() {}

    @Override 
    public void execute() {
        double pullieAbsPos = mCage.getPullieAbsolutePos();
        double liftyAbsPos = mCage.getLiftyAbsolutePos();
        double pullieDesiredPos = mCage.getPullieDesiredPos();
        double liftyUpPos = mCage.getLiftyUpPos();

        double pulliePower = Math.abs(pullieAbsPos - pullieDesiredPos) < pulliePosDeadzone ? 0 : 1;
        pulliePower *= pullieDesiredPos < pullieAbsPos ? -1 : 1;

        double liftyPower = Math.abs(liftyAbsPos - liftyUpPos) < liftyPosDeadzone ? 0 : 1;
        liftyPower *= liftyUpPos < liftyAbsPos ? -1 : 1;
        
        mCage.setPulliePower(pulliePower);
        mCage.setLiftyPower(liftyPower);
        
        // Telemetry
        SmartDashboard.putNumber("Pullie abs pos", pullieAbsPos);
        SmartDashboard.putNumber("Lifty abs pos", liftyAbsPos);
        SmartDashboard.putNumber("Pullie Desired Pos", pullieDesiredPos);
        SmartDashboard.putNumber("Lifty Up pos", liftyUpPos);
        SmartDashboard.putNumber("CageDesiredPos", mCage.getPullieDesiredIndex());
    }

    @Override
    public void end(boolean isFinished) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
