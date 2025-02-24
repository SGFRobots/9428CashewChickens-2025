package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class ElevatorControl extends Command {
    private final Elevator mElevator;
    private final GenericHID mController;

    public ElevatorControl(Elevator pElevator, GenericHID pController) {
        mElevator = pElevator;
        mController = pController;
        addRequirements(mElevator);
    }

    @Override
    public void execute() {
        double joystick = -mController.getRawAxis(Constants.Controllers.selected.RightYPort);
        joystick = (Math.abs(joystick) < 0.01) ? 0 : joystick/2;
        mElevator.setPower(joystick);
    }

    @Override
    public void end(boolean interrupted) {
        mElevator.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
