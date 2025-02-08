package frc.robot.subsystems.algaeGround;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.AnalogInput;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeGroundSubsystem extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public static final double drivePosition=0;
  public static final double processorPosition=-10;
  public static final double intakePosition=-60;



  //add a timer object
  private Timer timer = new Timer();
 //add anew sparkmax brushless
  private SparkMax algaeMotor = new SparkMax(8, MotorType.kBrushless);
  //add new victor spx
  //private WPI_VictorSPX victor = new WPI_VictorSPX(0);
  private SparkMaxConfig algaeConfig=new SparkMaxConfig();
  private SparkClosedLoopController algaePIDController=algaeMotor.getClosedLoopController();
  public RelativeEncoder algaeEncoder=algaeMotor.getEncoder();

  
    AnalogInput analogInput = new AnalogInput(0);
  
    // The full range of motion (in meaningful external units) is 0-180 (this could be degrees, for instance)
    // The "starting point" of the motion, i.e. where the mechanism is located when the potentiometer reads 0v, is 30.
    AnalogPotentiometer pot = new AnalogPotentiometer(analogInput, 300.0, 30.0);

  public AlgaeGroundSubsystem(){
    timer.start();
    //  and enables 2-bit averaging
    analogInput.setAverageBits(2);
  }
  
  

  @Override
  public void periodic() {
    // Put code here to be run every loop
    if(timer.hasElapsed(2.0)) {
      System.out.println(pot.get());
      timer.reset();
    }
  }
  
}
