package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Coral;

public class CoralScore extends Command {
    private final Coral mCoral;
    private final GenericHID mController;
    private boolean startWithCoral;
    private double power;

    public CoralScore(Coral pCoral, GenericHID pController) {
        mCoral = pCoral;
        mController = pController;
        power = 1;

        addRequirements(mCoral);
    }
    
    @Override
    public void initialize() {
        // startWithCoral = mCoral.coralIn;
        // power = startWithCoral ? -0.5 : 0.5;
    }

    @Override 
    public void execute() {
        double joystick = -mController.getRawAxis(Constants.Controllers.selected.SwitchF);
        joystick = (Math.abs(joystick) < 0.01) ? 0 : joystick/2;
        mCoral.setPower(joystick/2);
    }

    @Override
    public void end(boolean isFinished) {}

    @Override
    public boolean isFinished() {
        // return startWithCoral != mCoral.coralIn;
        return false;
    }
}
