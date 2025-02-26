package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Algae;

public class AlgaeWheel extends Command{
    private final Algae mAlgae; 
    private final GenericHID mController;

    public AlgaeWheel(Algae pAlgae, GenericHID pController) {
        mAlgae = pAlgae;
        mController = pController;

        addRequirements(mAlgae);
    }

    @Override
    public void initialize() {}

    @Override 
    public void execute() {
        double dial = mController.getRawAxis(Constants.Controllers.selected.SwitchE);
        dial = Math.abs(dial) < 0.1 ? 0 : dial;
        mAlgae.setWheelPower(dial);
        SmartDashboard.putNumber("dial", dial);
        mAlgae.telemetry();
    }

    @Override
    public void end(boolean isFinished) {}

    @Override
    public boolean isFinished() {return false;}
}
