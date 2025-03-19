package frc.robot.commands.Arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Cage;

public class CageControl extends Command {
    private final Cage mCage;
    private final PIDController posPID;

    public CageControl(Cage pCage) {
        mCage = pCage;
        posPID = new PIDController(0.08, 0, 0);
        addRequirements(mCage);
    }

    @Override
    public void initialize() {}

    @Override 
    public void execute() {
        double power = posPID.calculate(mCage.getRelativePos(), mCage.getDesiredPos());
        mCage.setPower(power);
        SmartDashboard.putNumber("PID error", posPID.getError());
        // System.out.println(mCage.getAbsolutePos());
        SmartDashboard.putNumber("Cage rel pos", mCage.getAbsolutePos());
    }

    @Override
    public void end(boolean isFinished) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
