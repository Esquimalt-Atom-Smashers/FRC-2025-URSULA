package frc.robot.subsystems.algaeGround;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.AnalogInput;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeGroundSubsystem extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  //degrees
  public static final double DRIVE_POSITION=0;
  public static final double PROCESSOR_POSIITON=-10;
  public static final double INTAKE_POSITION=-60;
  public static final double GEARBOX_RATIO = 100;
  public static final double GEARCHAIN_RATIO = 42/18;
  public static final double POT_OFFSET = 120;//TODO






  //add a timer object
  private Timer timer = new Timer();
 //add motors
  private SparkMax algaeArmMotor = new SparkMax(2, MotorType.kBrushless);
  private SparkMaxConfig algaeArmConfig=new SparkMaxConfig();
  private SparkClosedLoopController algaeArmPIDController=algaeArmMotor.getClosedLoopController();
  public RelativeEncoder algaeArmEncoder=algaeArmMotor.getEncoder();
  AnalogInput analogInput = new AnalogInput(0);
  AnalogPotentiometer potentiometer = new AnalogPotentiometer(analogInput, -300.0, POT_OFFSET);

  public AlgaeGroundSubsystem(){
    timer.start();
    analogInput.setAverageBits(2);
    //Algae Arm motor encoder to degrees conversion
    algaeArmConfig.encoder.positionConversionFactor(360/(GEARBOX_RATIO*GEARCHAIN_RATIO))
    .velocityConversionFactor(360/(GEARBOX_RATIO*GEARCHAIN_RATIO));
    
    //set the initial position of the motor using the potentiometer
    algaeArmMotor.getEncoder().setPosition(potentiometer.get());

    //PID and power Settings
    algaeArmConfig.smartCurrentLimit(1,3,200);
    algaeArmConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .p(0.1).i(0.000).d(0.01).maxOutput(0.2).minOutput(-0.2);


    algaeArmMotor.configure(algaeArmConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    algaeArmPIDController.setReference(DRIVE_POSITION, SparkMax.ControlType.kPosition);  

  }

  public void setTargetAngle(double degrees){
    algaeArmPIDController.setReference(degrees,ControlType.kPosition);
  }
  
  
  @Override
  public void periodic() {
    // Put code here to be run every loop
    if(timer.hasElapsed(4.0)) {
      System.out.println("Algae angle = "+ potentiometer.get());
      timer.reset();
    }
  }
  
}
