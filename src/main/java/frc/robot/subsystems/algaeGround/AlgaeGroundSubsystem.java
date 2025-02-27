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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeGroundSubsystem extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  //degrees
  public static final double DRIVE_POSITION=15;
  public static final double PROCESSOR_POSIITON=-40;
  public static final double INTAKE_POSITION=-94;
  public static final double GEARBOX_RATIO = 100;
  public static final double GEARCHAIN_RATIO = 42/18;
  public static final double POT_OFFSET = 120;//TODO
  public static final double INTAKE_VOLTAGE =-10;
  public static final double OUTTAKE_VOLTAGE =5;
  //public static final double HOLD_VOLTAGE = INTAKE_VOLTAGE/50;


  //add a timer object
  private Timer timer = new Timer();
 //add motors
  private SparkMax algaeArmMotor = new SparkMax(2, MotorType.kBrushless);
  private SparkMax algaeIntakeMotor = new SparkMax(3, MotorType.kBrushless);
  private SparkMaxConfig algaeArmConfig = new SparkMaxConfig();
  private SparkMaxConfig algaeIntakeConfig = new SparkMaxConfig();
  private SparkClosedLoopController algaeArmPIDController = algaeArmMotor.getClosedLoopController();
  private SparkClosedLoopController algaeIntakePIDController = algaeIntakeMotor.getClosedLoopController();
  public RelativeEncoder algaeArmEncoder = algaeArmMotor.getEncoder();
  AnalogInput analogInput = new AnalogInput(0);
  AnalogPotentiometer potentiometer = new AnalogPotentiometer(analogInput, -300.0, POT_OFFSET);

  public AlgaeGroundSubsystem(){
    timer.start();
    analogInput.setAverageBits(2);
    //Algae Arm motor encoder to degrees conversion
    algaeArmConfig.encoder.positionConversionFactor(360/(GEARBOX_RATIO*GEARCHAIN_RATIO))
    .velocityConversionFactor(360/(GEARBOX_RATIO*GEARCHAIN_RATIO));
    
    

    //PID and power Settings
    algaeArmConfig.smartCurrentLimit(1,3,200);
    algaeArmConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .p(0.1).i(0.000).d(0.01).maxOutput(0.3).minOutput(-0.2);

    algaeIntakeConfig.smartCurrentLimit(2,4,2000);
    algaeIntakeConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    algaeIntakeConfig.closedLoop.p(0.001).i(0.000).d(0.0001).maxOutput(0.2).minOutput(-0.2);
    algaeIntakeConfig.closedLoop.p(0.01, ClosedLoopSlot.kSlot1);


    algaeArmMotor.configure(algaeArmConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    algaeIntakeMotor.configure(algaeIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    //set the initial position of the motor using the potentiometer
    algaeArmMotor.getEncoder().setPosition(potentiometer.get());
    algaeArmPIDController.setReference(DRIVE_POSITION, SparkMax.ControlType.kPosition);  
    algaeIntakePIDController.setReference(0, SparkMax.ControlType.kVoltage);

  }

  protected void setArmAngle(double degrees){
    algaeArmPIDController.setReference(degrees,ControlType.kPosition);
  }
  


  private void intake(){
    algaeIntakePIDController.setReference(INTAKE_VOLTAGE, ControlType.kVoltage);
  }
  private void outtake(){
    algaeIntakePIDController.setReference(OUTTAKE_VOLTAGE, ControlType.kVoltage);
  }
  private void hold(){
    algaeIntakeMotor.getEncoder().setPosition(0);
    algaeIntakePIDController.setReference(-5, ControlType.kPosition);
  }

  private void stop(){
    algaeIntakePIDController.setReference(0, ControlType.kVoltage);
  }
  public Command holdCommand() {
    return Commands.runOnce(() -> {hold();}, this);}


  public Command intakeUntilStalledCommand() {
    return Commands.runOnce(() -> {intake();}, this)
      .andThen(Commands.waitUntil(() -> {//wait for motor to spin up
        System.out.println("Algae Intake Velocity = "+ algaeIntakeMotor.getEncoder().getVelocity());
        return algaeIntakeMotor.getEncoder().getVelocity()<=-2000;}))
      .andThen(Commands.waitUntil(() -> {//wiat for the algae ball to stop the motor
        if (algaeIntakeMotor.getEncoder().getVelocity()>=-200){
          hold();
          return true;
        } else {
          return false;
        }}))
      .withName("Intake wait for Algae");
    }
  public Command outtakeCommand(){
    return Commands.runOnce(() -> {outtake();}, this)    
    .withName("Outtake Algae");
  }
  
  public SequentialCommandGroup intakeSequenceCommand(){
    return new SequentialCommandGroup(
      new AlgaeToPosCommand(INTAKE_POSITION,this),
      intakeUntilStalledCommand(),
      new AlgaeToPosCommand(DRIVE_POSITION,this)
    );
  }
  public SequentialCommandGroup stopIntakeSequenceCommand(){
    return new SequentialCommandGroup(
      new AlgaeToPosCommand(DRIVE_POSITION,this),
      holdCommand()
    );
  }
  
  
  @Override
  public void periodic() {
    // Put code here to be run every loop
    if(timer.hasElapsed(4.0)) {
      System.out.println("Algae angle = "+ potentiometer.get());
      timer.reset();
    }
  }
  

  // Testing Commands

}
