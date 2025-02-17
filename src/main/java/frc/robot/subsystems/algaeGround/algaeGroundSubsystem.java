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
  public static final double drivePosition=0;
  public static final double processorPosition=-10;
  public static final double intakePosition=-60;
  public static final double GEARBOX_RATIO = 100;
  public static final double GEARCHAIN_RATIO = 42/18;
  public static final double POT_OFFSET = 120;//TODO




  //add a timer object
  private Timer timer = new Timer();
 //add anew sparkmax brushless
  private SparkMax algaeMotor = new SparkMax(2, MotorType.kBrushless);
  //add new victor spx
  //private WPI_VictorSPX victor = new WPI_VictorSPX(0);
  private SparkMaxConfig algaeConfig=new SparkMaxConfig();
  private SparkClosedLoopController algaePIDController=algaeMotor.getClosedLoopController();
  public RelativeEncoder algaeEncoder=algaeMotor.getEncoder();

  
    AnalogInput analogInput = new AnalogInput(0);
  
    // The full range of motion (in meaningful external units) is 0-180 (this could be degrees, for instance)
    // The "starting point" of the motion, i.e. where the mechanism is located when the potentiometer reads 0v, is 30.
    AnalogPotentiometer pot = new AnalogPotentiometer(analogInput, -300.0, POT_OFFSET);

  public AlgaeGroundSubsystem(){
    timer.start();
    //  and enables 2-bit averaging
    analogInput.setAverageBits(2);
    algaeConfig.encoder.positionConversionFactor(360/(GEARBOX_RATIO*GEARCHAIN_RATIO))
    .velocityConversionFactor(360/(GEARBOX_RATIO*GEARCHAIN_RATIO));
    algaeConfig.smartCurrentLimit(1,3,200);
    algaeMotor.getEncoder().setPosition(pot.get());
    algaeConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .p(0.1).i(0.000).d(0.01).maxOutput(0.2).minOutput(-0.2);
    algaeMotor.configure(algaeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    algaePIDController.setReference(drivePosition, SparkMax.ControlType.kPosition);  

  }

  public void setTargetPosition(double degrees){
    algaePIDController.setReference(degrees,ControlType.kPosition);
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
