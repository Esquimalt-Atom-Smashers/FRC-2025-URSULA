package frc.robot.subsystems.algaeremover;

import edu.wpi.first.wpilibj2.command.Command;

public class SpinAlgaeSpinnyThing extends Command{
    private AlgaeRemoverSubsystem algaeRemoverSubsystem;
    private double targetSpeed;

    public SpinAlgaeSpinnyThing(AlgaeRemoverSubsystem algaeRemoverSubsystem, double targetSpeed) {
        this.algaeRemoverSubsystem = algaeRemoverSubsystem;
        this.targetSpeed = targetSpeed;
    }

    public SpinAlgaeSpinnyThing(AlgaeRemoverSubsystem algaeRemoverSubsystem, AlgaeRemoverSubsystem.SpinnySpeeds targetSpeed) {
        this.algaeRemoverSubsystem = algaeRemoverSubsystem;
        this.targetSpeed = targetSpeed.value;
    }

    @Override
    public void initialize() {
        algaeRemoverSubsystem.setSpinnySpeed(0);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(algaeRemoverSubsystem.getSpinnySpeed() - targetSpeed) < 1.0;
    }
}
