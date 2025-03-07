package frc.robot.commands.Arm;


import frc.robot.Constants;
import frc.robot.subsystems.Algae;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AlgaeControl extends Command {
    private final Algae mAlgae;
    private final GenericHID mController;
    private boolean open;
    private boolean shoot;
    private final double originalPos;
    private int previousPOV;

    public AlgaeControl(Algae pAlgae, GenericHID pController) {
        // Initialize variables
        mAlgae = pAlgae;
        mController = pController;

        // Reset state of motors
        open = false;
        shoot = false;
        originalPos = mAlgae.getPosition();

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
        } else if (pov == Constants.Controllers.XBox.DPadUp) {
            // Shoot out algae
            open = false;
            shoot = true;
        } else if (pov == Constants.Controllers.XBox.DPadRight && pov != previousPOV) {
            // Disable all motors
            mAlgae.stop();
            open = false;
            shoot = false; 
        } else {
            shoot = false;
        }

        // Set controller inputs
        previousPOV = pov;
        
        // Intake
        if (open && mAlgae.getPosition() < originalPos+0.3) {
            // Go up high power
            mAlgae.setPosPower(0.1);
        } else if (open && mAlgae.getPosition() < originalPos + 0.5) {
            // Holding power
            mAlgae.setPosPower(0.05);
        } else {
            // Go down / stop
            mAlgae.setPosPower(0);
        }

        // Wheel control
        if (open) {
            // Hold in
            mAlgae.setWheelPower(-0.8);
        } else if (shoot) {
            // Shoot out
            mAlgae.setWheelPower(0.8);
        } else {
            // Stop
            mAlgae.setWheelPower(0);
        }

        // Stop wheels when low position
        if (mAlgae.getPosition() < originalPos + 0.2){
            mAlgae.setWheelPower(0);
            shoot = false;
        }

        // Telemetry
        SmartDashboard.putBoolean("Algeastate", open);
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