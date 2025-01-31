package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Coral;


public class CoralIntake extends Command{
    private final Coral mCoral;

    public CoralIntake(Coral pCoral) {
        mCoral = pCoral;
    }

    @Override
    public void initialize() {}

    @Override 
    public void execute() {}

    @Override
    public void end(boolean isFinished) {}

    @Override
    public boolean isFinished() {return false;}
}
