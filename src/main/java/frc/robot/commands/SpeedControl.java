package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class SpeedControl {
    public final SlowMode slow;
    public final FastMode fast;

    public SpeedControl(SwerveSubsystem subsystem) {
        slow = new SlowMode(subsystem);
        fast = new FastMode(subsystem);
    }

    public class SlowMode extends Command {
        private final SwerveSubsystem mSubsystem;

        public SlowMode(SwerveSubsystem subsystem) {
            mSubsystem = subsystem;
        }

        @Override
        public void initialize() {
            mSubsystem.toggleSlowMode(true);
        }
    
        @Override
        public void execute() {}
    
        @Override
        public void end(boolean isFinished) {
            mSubsystem.toggleSlowMode(false);
        }
    
        @Override
        public boolean isFinished() {
            return false;
        }
    }

    public class FastMode extends Command {
        private final SwerveSubsystem mSubsystem;

        public FastMode(SwerveSubsystem subsystem) {
            mSubsystem = subsystem;
        }

        @Override
        public void initialize() {
            mSubsystem.toggleFastMode(true);
        }
    
        @Override
        public void execute() {}
    
        @Override
        public void end(boolean isFinished) {
            mSubsystem.toggleFastMode(false);
        }
    
        @Override
        public boolean isFinished() {
            return false;
        }
    }
}
