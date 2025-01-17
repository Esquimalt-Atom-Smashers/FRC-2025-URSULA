package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class TestSubsystem extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  //add a timer object
  private Timer timer = new Timer();

  //add a new spark controller
  //private Spark spark = new Spark(0);
  //add anew sparkmax brushless
  //private SparkMax sparkmax = new SparkMax(3, MotorType.kBrushless);
  //add new victor spx
  //private WPI_VictorSPX victor = new WPI_VictorSPX(0);

 
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
