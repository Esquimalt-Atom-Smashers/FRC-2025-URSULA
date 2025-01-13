package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class TestSubsystem extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  //add a timer object
  private Timer timer = new Timer();
 
  public TestSubsystem() {
    // Initialize the subsystem here
    timer.start();
  }

  @Override
  public void periodic() {
    // Put code here to be run every loop
    if(timer.hasElapsed(1.0)) {
      System.out.println("Hello World");
      timer.reset();
    }
  }

}
