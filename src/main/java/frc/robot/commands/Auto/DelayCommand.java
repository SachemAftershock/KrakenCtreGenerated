package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;

public class DelayCommand extends Command {

    final boolean showPrints = false;		

    private double mSecondsToDelay;
    private Timer mTimer;

    public DelayCommand(double theSecondsToDelay) {
        mSecondsToDelay = theSecondsToDelay;
        mTimer = new Timer();
        addRequirements();
    }   

    @Override
    public void initialize() {
        if (showPrints) System.out.println("DelayCommand started " + Double.toString(mSecondsToDelay) + " seconds.");
        mTimer.reset();
        mTimer.start();
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        
        if (mTimer.hasElapsed(mSecondsToDelay)) {
            if (showPrints) System.out.println("DelayCommand completed " + Double.toString(mSecondsToDelay) + " seconds.");
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        mTimer.stop();
    }
}

