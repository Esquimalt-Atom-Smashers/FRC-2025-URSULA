// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;


import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.elevator.ElevatorHomingCommand;
import frc.robot.commands.AutoPickup;
import frc.robot.commands.AutoPlace;
import frc.robot.commands.AutoPlace.Node;
import frc.robot.subsystems.algaeGround.AlgaeGroundSubsystem;
import frc.robot.subsystems.algaeGround.AlgaeToPosCommand;
import frc.robot.subsystems.coraldoor.CoralDoorSubsystem;
import frc.robot.subsystems.coraldoor.CoralDoorToPositionCommand;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorToPosCommand;
import frc.robot.subsystems.hangingmechanism.HangingSubsystem;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.swerveDrive.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerveDrive.TunerConstants;
import scoringcontroller.CommandCustomController;

public class RobotContainer {
    private double MaxSpeed = Math.min(1, TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)/3); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(.7).in(RadiansPerSecond); // 1/2 of a rotation per second max angular velocity
    private int level = 0;
    private AutoPlace.HexSide hexSide = AutoPlace.HexSide.A;
    private AutoPlace.Side side = AutoPlace.Side.one;
    private boolean level1Pickup = false;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    // private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
    //         .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandCustomController CustomController = new CommandCustomController(1);
    private final CommandXboxController operatorController = new CommandXboxController(2);
    private static final double XBOX_DEADBAND = 0.1;

    //Create Subsystems
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public LimelightSubsystem limelightSubsystem; 
    public final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    public final AlgaeGroundSubsystem algaeGroundSubsystem = new AlgaeGroundSubsystem();
    public final HangingSubsystem hangingSubsystem = new HangingSubsystem();
    public final CoralDoorSubsystem coralDoorSubsystem = new CoralDoorSubsystem();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        // Register the named commands for auto
        registerCommands();
        autoChooser = AutoBuilder.buildAutoChooser("ScoreL1FromCenter");
        SmartDashboard.putData("Auto Mode", autoChooser);
        limelightSubsystem = new LimelightSubsystem(drivetrain, true);

        configureBindings();
    }

    private void configureBindings() {
        //// ----------------- Driving Commands -----------------
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(MathUtil.applyDeadband(-driverController.getLeftY(),XBOX_DEADBAND) * MaxSpeed*(driverController.getRightTriggerAxis()*2+1)) // Drive forward with negative Y (forward)
                    .withVelocityY(MathUtil.applyDeadband(-driverController.getLeftX(), XBOX_DEADBAND) * MaxSpeed*(driverController.getRightTriggerAxis()*2+1)) // Drive left with negative X (left)
                    .withRotationalRate(MathUtil.applyDeadband(-driverController.getRightX(),XBOX_DEADBAND) * MaxAngularRate*(driverController.getRightTriggerAxis()*2+1)) // Drive counterclockwise with negative X (left)
            )
        );
        // Reset the field-centric heading on left bumper press
        driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));


        //// ----------------- Elevator Commands ----------------
        driverController.a().onTrue(new ElevatorToPosCommand(ElevatorSubsystem.lowPosition, elevatorSubsystem));
        driverController.b().onTrue(new ElevatorToPosCommand(ElevatorSubsystem.level2Position, elevatorSubsystem));
        driverController.x().onTrue(new ElevatorToPosCommand(ElevatorSubsystem.level3Position, elevatorSubsystem));
        driverController.y().onTrue(new ElevatorToPosCommand(ElevatorSubsystem.level4Position, elevatorSubsystem));


        //// --------------- Coral Door Commands ----------------
        driverController.leftTrigger(0.5).onTrue(new CoralDoorToPositionCommand(CoralDoorSubsystem.DoorPosition.OPEN, coralDoorSubsystem))
            .onFalse(new CoralDoorToPositionCommand(CoralDoorSubsystem.DoorPosition.CLOSED, coralDoorSubsystem));


        //// ----------------- Hanging Commands -----------------
        driverController.povUp().onTrue(hangingSubsystem.manualRetractCommand())
            .onFalse(hangingSubsystem.stopandZeroMotorCommand());
        driverController.povLeft().onTrue(hangingSubsystem.retractHangingMechanismCommand());
        driverController.povRight().onTrue(hangingSubsystem.extendHangingMechanismCommand());


        //// ------------------ Algae Commands ------------------
        driverController.rightBumper().onFalse(algaeGroundSubsystem.stopIntakeSequenceCommand())
            .onTrue(algaeGroundSubsystem.intakeSequenceCommand());
        //xboxController.a().onTrue(algaeGroundSubsystem.intakeUntilStalledCommand())
            //.onFalse(algaeGroundSubsystem.holdCommand());
        driverController.leftBumper().onTrue(algaeGroundSubsystem.outtakeCommand())
            .onFalse(algaeGroundSubsystem.holdCommand());
        driverController.povDown().onTrue(new AlgaeToPosCommand(AlgaeGroundSubsystem.PROCESSOR_POSIITON, algaeGroundSubsystem))
            .onFalse(new AlgaeToPosCommand(AlgaeGroundSubsystem.DRIVE_POSITION, algaeGroundSubsystem));

        
        //// ---------------- Automated Commands ----------------
        // Choosing where to score on Custom Controller
        CustomController.bt1().onTrue(new RunCommand(() -> {
            hexSide = AutoPlace.HexSide.A;
            side = AutoPlace.Side.one;
        }));
        CustomController.bt2().onTrue(new RunCommand(() -> {
            hexSide = AutoPlace.HexSide.A;
            side = AutoPlace.Side.two;
        }));
        CustomController.bt3().onTrue(new RunCommand(() -> {
            hexSide = AutoPlace.HexSide.B;
            side = AutoPlace.Side.one;
        }));
        CustomController.bt4().onTrue(new RunCommand(() -> {
            hexSide = AutoPlace.HexSide.B;
            side = AutoPlace.Side.two;
        }));
        CustomController.bt5().onTrue(new RunCommand(() -> {
            hexSide = AutoPlace.HexSide.C;
            side = AutoPlace.Side.one;
        }));
        CustomController.bt6().onTrue(new RunCommand(() -> {
            hexSide = AutoPlace.HexSide.C;
            side = AutoPlace.Side.two;
        }));
        CustomController.bt7().onTrue(new RunCommand(() -> {
            hexSide = AutoPlace.HexSide.D;
            side = AutoPlace.Side.one;
        }));
        CustomController.bt8().onTrue(new RunCommand(() -> {
            hexSide = AutoPlace.HexSide.D;
            side = AutoPlace.Side.two;
        }));
        CustomController.bt9().onTrue(new RunCommand(() -> {
            hexSide = AutoPlace.HexSide.E;
            side = AutoPlace.Side.one;
        }));
        CustomController.bt10().onTrue(new RunCommand(() -> {
            hexSide = AutoPlace.HexSide.E;
            side = AutoPlace.Side.two;
        }));
        CustomController.bt11().onTrue(new RunCommand(() -> {
            hexSide = AutoPlace.HexSide.F;
            level = 1;
        }));
        CustomController.bt12().onTrue(new RunCommand(() -> {
            hexSide = AutoPlace.HexSide.F;
            level = 2;
        }));
        CustomController.bt16().onTrue(new RunCommand(() -> {
            level = 1;
        }));
        CustomController.bt17().onTrue(new RunCommand(() -> {
            level = 2;
        }));
        CustomController.bt18().onTrue(new RunCommand(() -> {
            level = 3;
        }));
        CustomController.bt19().onTrue(new RunCommand(() -> {
            level = 4;
        }));

        // Autoplace command (Allow operator to also place)
        driverController.back().whileTrue(new AutoPlace(drivetrain, elevatorSubsystem, coralDoorSubsystem,
                                                                new Node(level, hexSide, side)));
        operatorController.rightBumper().whileTrue(new AutoPlace(drivetrain, elevatorSubsystem, coralDoorSubsystem,
                                                                new Node(level, hexSide, side)));

        // Auto pickup command
        // If wanting to pickup to score for level 1, press A, otherwise press Y
        operatorController.y().whileTrue(new RunCommand(() -> level1Pickup = false));
        operatorController.a().whileTrue(new RunCommand(() -> level1Pickup = true));
        operatorController.leftBumper().whileTrue(new AutoPickup(drivetrain, elevatorSubsystem, () -> AutoPickup.getCoralSide(drivetrain.getState().Pose), level1Pickup));

        // Commented out Bindings
        // hangingSubsystem.setDefaultCommand(new RunCommand(() -> 
        //     hangingSubsystem.setWinchPosition(hangingSubsystem.getWinchPosition() + xboxController.getRightY())
        // ));
        // xboxController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // xboxController.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-xboxController.getLeftY(), -xboxController.getLeftX()))
        // // ));
         // xboxController.pov(0).whileTrue(drivetrain.applyRequest(() ->
        //     forwardStraight.withVelocityX(0.5).withVelocityY(0))
        // );
        // xboxController.pov(180).whileTrue(drivetrain.applyRequest(() ->
        //     forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        // );
        // // Run SysId routines when holding back/start and X/Y.
        // // Note that each routine should be run exactly once in a single log.
        // xboxController.back().and(xboxController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // xboxController.back().and(xboxController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // xboxController.start().and(xboxController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // xboxController.start().and(xboxController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        //drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
    private void registerCommands(){
        // Register the commands here
        NamedCommands.registerCommand("ElevatorHomingCommand", new ElevatorHomingCommand(elevatorSubsystem));
        NamedCommands.registerCommand("AlgaeToDrivePos", new AlgaeToPosCommand(AlgaeGroundSubsystem.DRIVE_POSITION, algaeGroundSubsystem));
        NamedCommands.registerCommand("AlgaeToIntakePos", new AlgaeToPosCommand(AlgaeGroundSubsystem.INTAKE_POSITION, algaeGroundSubsystem));
        NamedCommands.registerCommand("ElevatorToLVL1", new ElevatorToPosCommand(ElevatorSubsystem.level1Position, elevatorSubsystem));
        NamedCommands.registerCommand("ElevatorToLVL2", new ElevatorToPosCommand(ElevatorSubsystem.level2Position, elevatorSubsystem));
        NamedCommands.registerCommand("ElevatorToLVL3", new ElevatorToPosCommand(ElevatorSubsystem.level3Position, elevatorSubsystem));
        NamedCommands.registerCommand("ElevatorToLVL4", new ElevatorToPosCommand(ElevatorSubsystem.level4Position, elevatorSubsystem));
        NamedCommands.registerCommand("OpenCoralDoor", new CoralDoorToPositionCommand(CoralDoorSubsystem.DoorPosition.OPEN, coralDoorSubsystem));
        NamedCommands.registerCommand("CloseCoralDoor", new CoralDoorToPositionCommand(CoralDoorSubsystem.DoorPosition.CLOSED, coralDoorSubsystem));
    }
}
