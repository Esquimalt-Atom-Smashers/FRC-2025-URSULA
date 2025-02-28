// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.jar.Attributes.Name;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.fasterxml.jackson.databind.util.Named;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
// import frc.robot.subsystems.Telemetry;
import frc.robot.subsystems.elevator.ElevatorHomingCommand;
import frc.robot.subsystems.algaeGround.AlgaeGroundSubsystem;
import frc.robot.subsystems.algaeGround.AlgaeToPosCommand;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorToPosCommand;
import frc.robot.subsystems.elevator.ElevatorToPosCommand;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.swerveDrive.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerveDrive.TunerConstants;

public class RobotContainer {
    private double MaxSpeed = Math.min(1, TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)/3); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(.7).in(RadiansPerSecond); // 1/2 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private static final double XBOX_DEADBAND = 0.1;


    //Create Subsystems
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public LimelightSubsystem limelightSubsystem; 
    public final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    public final AlgaeGroundSubsystem algaeGroundSubsystem = new AlgaeGroundSubsystem();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        //register the named commands for auto
        registerCommands();
        autoChooser = AutoBuilder.buildAutoChooser("MoveOnly");
        SmartDashboard.putData("Auto Mode", autoChooser);
        limelightSubsystem = new LimelightSubsystem(drivetrain, true);

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(MathUtil.applyDeadband(-joystick.getLeftY(),XBOX_DEADBAND) * MaxSpeed*(joystick.getRightTriggerAxis()*2+1)) // Drive forward with negative Y (forward)
                    .withVelocityY(MathUtil.applyDeadband(-joystick.getLeftX(), XBOX_DEADBAND) * MaxSpeed*(joystick.getRightTriggerAxis()*2+1)) // Drive left with negative X (left)
                    .withRotationalRate(MathUtil.applyDeadband(-joystick.getRightX(),XBOX_DEADBAND) * MaxAngularRate*(joystick.getRightTriggerAxis()*2+1)) // Drive counterclockwise with negative X (left)
            )
        );
        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // // ));
         // joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
        //     forwardStraight.withVelocityX(0.5).withVelocityY(0))
        // );
        // joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
        //     forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        // );

        // // Run SysId routines when holding back/start and X/Y.
        // // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        //joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // Algae Ground Testing Controls
        joystick.rightBumper().onFalse(algaeGroundSubsystem.stopIntakeSequenceCommand())
            .onTrue(algaeGroundSubsystem.intakeSequenceCommand());
        //joystick.a().onTrue(algaeGroundSubsystem.intakeUntilStalledCommand())
            //.onFalse(algaeGroundSubsystem.holdCommand());
        joystick.leftBumper().onTrue(algaeGroundSubsystem.outtakeCommand())
            .onFalse(algaeGroundSubsystem.holdCommand());
        joystick.pov(180).onTrue(new AlgaeToPosCommand(algaeGroundSubsystem.PROCESSOR_POSIITON, algaeGroundSubsystem))
            .onFalse(new AlgaeToPosCommand(algaeGroundSubsystem.DRIVE_POSITION, algaeGroundSubsystem));
        


        //Elevator Testing Controls
         joystick.a().onTrue(new ElevatorToPosCommand(ElevatorSubsystem.lowPosition, elevatorSubsystem));
         joystick.b().onTrue(new ElevatorToPosCommand(ElevatorSubsystem.level2Position, elevatorSubsystem));
         joystick.x().onTrue(new ElevatorToPosCommand(ElevatorSubsystem.level3Position, elevatorSubsystem));
         joystick.y().onTrue(new ElevatorToPosCommand(ElevatorSubsystem.level4Position, elevatorSubsystem));

       

        //drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
    private void registerCommands(){
        //register the commands here
        NamedCommands.registerCommand("ElevatorHomingCommand", new ElevatorHomingCommand(elevatorSubsystem));
        NamedCommands.registerCommand("AlgaeToDrivePos", new AlgaeToPosCommand(AlgaeGroundSubsystem.DRIVE_POSITION, algaeGroundSubsystem));
        NamedCommands.registerCommand("AlgaeToIntakePos", new AlgaeToPosCommand(AlgaeGroundSubsystem.INTAKE_POSITION, algaeGroundSubsystem));
        NamedCommands.registerCommand("ElevatorToLVL1", new ElevatorToPosCommand(ElevatorSubsystem.level1Position, elevatorSubsystem));
        NamedCommands.registerCommand("ElevatorToLVL2", new ElevatorToPosCommand(ElevatorSubsystem.level2Position, elevatorSubsystem));
        NamedCommands.registerCommand("ElevatorToLVL3", new ElevatorToPosCommand(ElevatorSubsystem.level3Position, elevatorSubsystem));
        NamedCommands.registerCommand("ElevatorToLVL4", new ElevatorToPosCommand(ElevatorSubsystem.level4Position, elevatorSubsystem));

    }
}
