package frc.robot.subsystems.hangingmechanism;

import edu.wpi.first.wpilibj2.command.Command;

public class WinchManualTestingCommand extends Command {
    private HangingSubsystem hangingSubsystem;
    private double targetPosition;

    public WinchManualTestingCommand(HangingSubsystem hangingSubsystem, double speed) {
        this.hangingSubsystem = hangingSubsystem;
        targetPosition = speed;
    }

   

    @Override
    public void initialize() {
        hangingSubsystem.setWinchPosition(targetPosition);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(hangingSubsystem.getWinchPosition() - targetPosition) < 1.0;
    }
}
