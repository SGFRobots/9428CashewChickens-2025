package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import java.util.function.DoubleSupplier;

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
        mElevator.setPower(0);
    }

    @Override
    public void end(boolean interrupted) {
        mElevator.stop();
    }
}
