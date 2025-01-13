package frc.robot.subsystems.limelight;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerveDrive.CommandSwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.Command;



public class LimelightSubsystem extends SubsystemBase{
    static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    static NetworkTableEntry Tx = table.getEntry("tx");
    static NetworkTableEntry Ty = table.getEntry("ty");
    static NetworkTableEntry Ta = table.getEntry("ta");
    static NetworkTableEntry Pipeline = table.getEntry("pipeline");
    static NetworkTableEntry Tv = table.getEntry("tv"); //are there any valid targets
    private CommandSwerveDrivetrain drivetrain;
    private boolean useLimelightForPose = false;
    private Timer printTimer=new Timer();
    public LimelightSubsystem(CommandSwerveDrivetrain drivetrain, boolean useLimelightForPose){
        this.drivetrain=drivetrain;
        this.useLimelightForPose=useLimelightForPose;
        printTimer.start();
    }
    
    public double[] getTargetPose(){
        double pose[]= table.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
        SmartDashboard.putNumber("pose0 ", pose[0]);
        SmartDashboard.putNumber("pose1", pose[1]);
        SmartDashboard.putNumber("pose2", pose[2]);
        SmartDashboard.putNumber("pose3", pose[3]);
        SmartDashboard.putNumber("pose4", pose[4]);
        SmartDashboard.putNumber("pose5", pose[5]);
        return pose;
        
    }
    
    public  double[] getTargets(){
        //Brandon Feb 15: I solved the error you were having with the smartDashboard. 
        //It needs to be called within a method, not within the construction of the Class.
        //you should probably move this into the Subsystems folder too.
        
        //read values periodically
        double x = Tx.getDouble(0.0);
        double y = Ty.getDouble(0.0);
        double area = Ta.getDouble(0.0);
        double[] returnArray= {x,y,area};

        //post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);

        return returnArray;
    }
    public void setPipeline(int m_pipeLine){
        Pipeline.setNumber(m_pipeLine);
    } 
    public boolean hasTargets(){
        boolean returnBool =false;
        if((double)Tv.getNumber(0)>0){//for some reason tv.getnumber must be mapped to double, not int or boolean
            returnBool=true;
        }
        return returnBool;
    }


    public Command getTargetsCommand() {
        return runOnce(()->getTargets());
    }
    public Command checkForTargetsCommand() {
        return runOnce( ()->hasTargets());
    }
    public Command setPipeline9Command() {
        return runOnce( ()->this.setPipeline(9));
    }

    public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> setPipeline(0));
    };
    
    @Override
    public void periodic() {
        /*
     * This example of adding Limelight is very simple and may not be sufficient for on-field use.
     * Users typically need to provide a standard deviation that scales with the distance to target
     * and changes with number of tags available.
     *
     * This example is sufficient to show that vision integration is possible, though exact implementation
     * of how to use vision should be tuned per-robot and to the team's specification.
     */
    if (useLimelightForPose) {
      var driveState = drivetrain.getState();
      double headingDeg = driveState.Pose.getRotation().getDegrees();
      double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

      LimelightHelpers.SetRobotOrientation("limelight", headingDeg, 0, 0, 0, 0, 0);
      var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
      if (llMeasurement != null && llMeasurement.tagCount > 0 && omegaRps < 2.0) {
        drivetrain.addVisionMeasurement(llMeasurement.pose, Utils.fpgaToCurrentTime(llMeasurement.timestampSeconds));
      }
    } else if(printTimer.hasElapsed(1)){
        printTimer.reset();
        System.out.println("Limelight not updatingPose");

    }
        
    }
    
}