package frc.robot.commands.Arm;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.subsystems.Algae;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AlgaeControl extends Command {
    private final Algae mAlgae;
    private final GenericHID mController;
    private boolean open;
    private boolean holding;
    private boolean shoot;
    private final double originalPos;
    private final PIDController PID;
    private int previousPOV;

    public AlgaeControl(Algae pAlgae, GenericHID pController) {
        mAlgae = pAlgae;
        mController = pController;
        open = false;
        holding = false;
        shoot = false;
        originalPos = mAlgae.getPosition();
        PID = new PIDController(0.25, 0, 0);
        previousPOV = -1;

        addRequirements(pAlgae);
    }

    @Override
    public void initialize() {
    }

    @Override 
    public void execute() {
        int pov = mController.getPOV();

        if (pov == 270 && pov != previousPOV) {
            // open
            open  = true;
            holding = true;
            shoot = false;
            previousPOV = 270;

        } else if (pov == 0 && pov != previousPOV) {
            // outtake
            open = false;
            holding = false;
            shoot = true;
            previousPOV = 0;
            mAlgae.setWheelPower(0.8);
            
        } else if (pov == 180 && pov != previousPOV) {
            // keep
            open = false;
            holding = false;
            shoot = false;
            // holding = !holding;
            // if (mAlgae.getPosition() > originalPos + 0.2) {
            //     mAlgae.setWheelPower(-0.5);
            //     // originalPos = mAlgae.getPosition()
            // } 
            previousPOV = 180;

        }
        else if(pov == 90 && pov != previousPOV){
            mAlgae.stop();
            open = false;
            holding = false;
            shoot = false; 
            previousPOV = 90;

        }
        
        if (open && mAlgae.getPosition() < originalPos+0.3) {
            mAlgae.setPosPower(0.1);
        } else if (open && mAlgae.getPosition() < originalPos + 0.5) {
            mAlgae.setPosPower(0.05);
        } else {
            mAlgae.setPosPower(0);
        }
        if (holding) {
            mAlgae.setWheelPower(-0.8);
        } else if (!shoot) {
            mAlgae.setWheelPower(0);
        }
        if (mAlgae.getPosition() < originalPos + 0.2){
            mAlgae.setWheelPower(0);
        }

        SmartDashboard.putNumber("Algeapos", mAlgae.getPosition() - originalPos);
        SmartDashboard.putBoolean("Algeastate", open);
    }

    @Override
    public void end(boolean isFinished) {
        mAlgae.stop();
        open = false;
        holding = false;
    }

    @Override
    public boolean isFinished() {
        // if (timer.get() > 1){
        //     return true;
        // }
        return false;
    }
}