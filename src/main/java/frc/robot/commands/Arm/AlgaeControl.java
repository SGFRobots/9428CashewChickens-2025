package frc.robot.commands.Arm;


import frc.robot.Constants;
import frc.robot.subsystems.Algae;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AlgaeControl extends Command {
    private final Algae mAlgae;
    private final GenericHID mController;
    private final PIDController PID;
    private boolean open;
    private boolean shoot;
    private final double originalPos;
    private int previousPOV;

    public AlgaeControl(Algae pAlgae, GenericHID pController) {
        // Initialize variables
        mAlgae = pAlgae;
        mController = pController;
        PID = new PIDController(0.03, 0, 0);

        // Reset state of motors
        open = false;
        shoot = false;
        originalPos = mAlgae.getAbsolutePos();

        // Reset controller inputs
        previousPOV = -1;

        addRequirements(pAlgae);
    }

    @Override
    public void initialize() {
    }

    @Override 
    public void execute() {
        // Get controller input
        int pov = mController.getPOV();

        // Sets algae behavior based on DPad location
        if (pov == Constants.Controllers.XBox.DPadDown && pov != previousPOV) {
            // Open up algae
            open  = !open;
            shoot = false;
            // Set desired position of position motor
            if (open) {
                mAlgae.setDesiredPos(1);
            } else {
                mAlgae.setDesiredPos(0);
            }
        } else if (pov == Constants.Controllers.XBox.DPadUp) {
            // Shoot out algae
            open = false;
            shoot = true;
            mAlgae.setDesiredPos(2);
        } else if (pov == Constants.Controllers.XBox.DPadRight && pov != previousPOV) {
            // Disable all motors
            mAlgae.stop();
            open = false;
            shoot = false; 
            mAlgae.setDesiredPos(0);
        } else {
            shoot = false;
        }

        // Set controller inputs
        previousPOV = pov;
        

        // Set position motor power to the desired position
        double power = PID.calculate(mAlgae.getAbsolutePos(), mAlgae.getDesiredPos());
        // power = (power < 0) ? 0 : power;
        mAlgae.setPosPower(power);

        // Wheel control
        if (open) {
            // Hold in
            mAlgae.setWheelPower(-0.8);
        } else if (shoot) {
            // Shoot out
            mAlgae.setWheelPower(1);
        } else {
            // Stop
            mAlgae.setWheelPower(0);
        }

        // Telemetry
        // SmartDashboard.putNumber("Algae Abs Pos", mAlgae.getAbsolutePos());
        SmartDashboard.putNumber("Algae Pos", mAlgae.getRelativePos());
    }

    @Override
    public void end(boolean isFinished) {
        // Stops all algae motors 
        mAlgae.stop();
        open = false;
        shoot = false;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}