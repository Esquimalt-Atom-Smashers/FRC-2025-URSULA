package frc.robot.subsystems.hangingmechanism;

import edu.wpi.first.wpilibj2.command.Command;

public class WinchToPositionCommand extends Command {
    private HangingSubsystem hangingSubsystem;
    private double targetPosition;

    public WinchToPositionCommand(HangingSubsystem hangingSubsystem, HangingSubsystem.WinchPosition targetPosition) {
        this.hangingSubsystem = hangingSubsystem;
        this.targetPosition = targetPosition.value;
    }

    public WinchToPositionCommand(HangingSubsystem hangingSubsystem, double targetPosition) {
        this.hangingSubsystem = hangingSubsystem;
        this.targetPosition = targetPosition;
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
