package frc.robot.subsystems.algaeremover;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class SpinAlgaeSpinnyThing extends Command{
    private AlgaeRemoverSubsystem algaeRemoverSubsystem;
    private double motorPower;
    //timer to spin up the motor
    private Timer timer = new Timer();

    public SpinAlgaeSpinnyThing(AlgaeRemoverSubsystem algaeRemoverSubsystem, double motorPower) {
        this.algaeRemoverSubsystem = algaeRemoverSubsystem;
        this.motorPower = motorPower;
    }

    public SpinAlgaeSpinnyThing(AlgaeRemoverSubsystem algaeRemoverSubsystem, AlgaeRemoverSubsystem.SpinnyPowers motorPower) {
        this(algaeRemoverSubsystem, motorPower.value);
    }

    @Override
    public void initialize() {
        algaeRemoverSubsystem.setSpinnyPower(motorPower);
        timer.reset();
        timer.start();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(1);
    }
}
