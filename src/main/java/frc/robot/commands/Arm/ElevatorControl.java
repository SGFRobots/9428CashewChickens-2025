package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class ElevatorControl extends Command {
    private final Elevator mElevator;
    private final GenericHID mController;

    public ElevatorControl(Elevator pElevator, GenericHID pController) {
        // Initialize variables
        mElevator = pElevator;
        mController = pController;
        addRequirements(mElevator);
    }

    @Override
    public void execute() {
        // Set power to elevator based on joystick position
        double joystick = mController.getRawAxis(Constants.Controllers.XBox.RightYPort);
        joystick = (Math.abs(joystick) < 0.01) ? 0 : joystick/2;
        if (joystick != 0){
            mElevator.setPower(joystick);
            mElevator.setDesiredPosition(mElevator.getPosition());
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
