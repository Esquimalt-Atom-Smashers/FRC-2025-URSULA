package frc.robot.commands;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.swerveDrive.CommandSwerveDrivetrain;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
// import frc.robot.utilities.lists.Constants;
import frc.robot.subsystems.elevator.ElevatorToPosCommand;

public class AutoPlace extends SequentialCommandGroup {

    public enum HexSide {
        A("A"),
        B("B"),
        C("C"),
        D("D"),
        E("E"),
        F("F");

        // public Pose2d waypoint;
        public String name;
        private HexSide(/*Pose2d waypoint,*/ String name) {
            // this.waypoint = waypoint;
            this.name = name;
        }
    }

    public enum Side {
        one("1"),
        two("2");
        public String name;
        private Side(String name) {
            this.name = name;
        }
    }

    public static class Node {
        public int level;
        public Side side;
        public HexSide hexSide;
        public Node(int level, HexSide hexSide, Side side) {
            this.level = level;
            this.side = side;
            this.hexSide = hexSide;
        }

        public String toString() {
            return "Hex: " + hexSide.name + ", Side: " + side.name + ", Lvl: " + level;
        }
    }

    // Create the constraints to use while pathfinding
    private PathConstraints constraints = new PathConstraints(
            3.0, 4.0,
            Units.degreesToRadians(270), Units.degreesToRadians(360));

    public AutoPlace(CommandSwerveDrivetrain drivetrain, ElevatorSubsystem elevator, Node node) {
        this(drivetrain, elevator, node, "");
    }

    public AutoPlace(CommandSwerveDrivetrain drivetrain, ElevatorSubsystem elevator, Node node, String suppliedPathName) {
        PathPlannerPath path;
        String pathName = "";
        // Name format is [side symbol][1/2] (e.g. A1, A2, B1, B2)
        pathName += node.hexSide.name;
        pathName += node.side.name;
        // If the node is A1, append 1 to the path name
        // if (node.level == SuperstructurePreset.L1) pathName += "1";
        try {
            path = PathPlannerPath.fromPathFile(suppliedPathName.isEmpty() ? pathName : suppliedPathName);
        } catch (Exception e) {
            e.printStackTrace();
            throw (new RuntimeException("Loaded a path that does not exist."));
        }

        // Command to move the robot to the desired position along a path, running until path is complete
        Command move = new ParallelDeadlineGroup(
            // new InstantCommand(() -> drivetrain.applyRequest(() -> new SwerveRequest.SwerveDriveBrake())).repeatedly().withDeadline(new WaitCommand(0.3)),
            // new ParallelDeadlineGroup(
                // AutoBuilder.pathfindToPoseFlipped(path.getStartingHolonomicPose().get(), constraints),
                // new ConditionalCommand(
                    // superstructure.setPreset(node.l),
                    // new ConditionalCommand(
                        // superstructure.setPreset(SuperstructurePreset.L4_INTERMEDIATE),
                        // new InstantCommand(() -> {}),
                        // () -> node.l == SuperstructurePreset.L4 || node.l == SuperstructurePreset.L3
                    // ),
                    // () -> node.l == SuperstructurePreset.L2
                // )
            // ),
            // new ParallelDeadlineGroup(
                // AutoBuilder.followPath(path),
                // superstructure.setPreset(
                    // node.l != SuperstructurePreset.MANUAL_OVERRIDE ? node.l
                    // : (node.scrub != SuperstructurePreset.MANUAL_OVERRIDE) ? node.scrub : SuperstructurePreset.STOW_UPPER
                // )
            // )

            // On the fly pathfinding to station, or follow the path if supplied
            new ConditionalCommand(
                AutoBuilder.pathfindThenFollowPath(path, constraints),
                AutoBuilder.followPath(path),
                () -> suppliedPathName.isEmpty()
            ),
            // Move the superstructure into a safe position for moving
            new ConditionalCommand(
                elevatorToLevel(node.level, elevator),
                elevatorToLevel(0, elevator),
                () -> node.level != 0
            )
        );
        // new EventTrigger("Align").onTrue(new InstantCommand(() -> CommandScheduler.getInstance().schedule(
            // superstructure.setPreset(node.l != SuperstructurePreset.MANUAL_OVERRIDE ? node.l : (node.scrub != SuperstructurePreset.MANUAL_OVERRIDE ? node.scrub : SuperstructurePreset.STOW_UPPER))
        // )));

        // Command to move elevator to the desired position and score the coral
        SequentialCommandGroup place = new SequentialCommandGroup(
            // Move elevator until at desired position, with a timeout
            elevatorToLevel(node.level, elevator)
                .until(new ElevatorToPosCommand(ElevatorSubsystem.level1Position, elevator)::isFinished)
                .withTimeout(1),
            // Wait some time if going to L3/L4
            new WaitCommand((node.level == 3) || (node.level == 4) ? 1 : 0),
            // // Shoot out the coral
            // new ParallelDeadlineGroup(
            //     // Run until the shoot sensors are cleared, or for a timeout if going to L1
            //     node.l == SuperstructurePreset.L1 ?
            //         new WaitCommand(2) :
            //         new WaitUntilCommand(superstructure.getCoralSensorIntake().negate().and(superstructure.getCoralSensorPlace().negate())),
            //     new ConditionalCommand(
            //         // Not going to L1; drive the shooter
            //         superstructure.setPreset(SuperstructurePreset.getCorrespondingGoState(node.l)),
            //         // Going to L1; run the L1 shoot sequence
            //         new SequentialCommandGroup(
            //             // Ready the coral for shooting (pulling it half-way in) with a timeout
            //             superstructure.setPresetWithFarSpit(SuperstructurePreset.L1).withTimeout(1),
            //             // Drive the shooter
            //             superstructure.setPreset(SuperstructurePreset.L1_GO)
            //         ),
            //         () -> node.l != SuperstructurePreset.L1
            //     )
            // )
            
            // Score Coral
            new ParallelDeadlineGroup(
                // Add command to drive right
                new InstantCommand(() -> drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityY(0.5))).repeatedly().withTimeout(1)
            )
        );

        if (!Utils.isSimulation()) {
            // Move the robot to desired position, stowing scrubber along the way
            addCommands(new ParallelDeadlineGroup(move));
            addCommands(place);

            // addCommands(superstructure.setPreset(SuperstructurePreset.STOW_UPPER).until(superstructure::atSetpoint).withTimeout(0.5));

            // Back up slightly while stowing the superstructure for a period of time
            Timer timer = new Timer();
            addCommands(
                new ParallelCommandGroup(
                    // Move the superstructure to the desired stow position
                    new ElevatorToPosCommand(ElevatorSubsystem.lowPosition, elevator),
                    // Adjust target speed to accelerate backwards (-X in robot centric)
                    new SequentialCommandGroup(
                        new InstantCommand(timer::restart),
                        new InstantCommand(() -> drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityX(-timer.get() * 4))).repeatedly()
                    )
                ).withDeadline(new WaitCommand(0.5))
            );
        } else {
            // Simplified place in simulation
            Timer timer = new Timer();
            addCommands(
                // Move the robot to desired position, stowing scrubber along the way
                new ConditionalCommand(
                    AutoBuilder.pathfindThenFollowPath(path, constraints),
                    AutoBuilder.followPath(path),
                    () -> suppliedPathName.isEmpty()
                ),

                /*
                 * No place or scrub in simulation
                 */

                // Adjust target speed to accelerate backwards (-X in robot centric) for a period of time
                new InstantCommand(timer::restart),
                new InstantCommand(() -> drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityX(-timer.get() * 4))).repeatedly().withDeadline(new WaitCommand(0.5))
            );
        }
    }

    public Command elevatorToLevel(int level, ElevatorSubsystem elevator) {
        if (level == 1) {
            return new ElevatorToPosCommand(ElevatorSubsystem.level1Position, elevator);
        }
        else if (level == 2) {
            return new ElevatorToPosCommand(ElevatorSubsystem.level2Position, elevator);
        }
        else if (level == 3) {
            return new ElevatorToPosCommand(ElevatorSubsystem.level3Position, elevator);
        }
        else if (level == 4) {
            return new ElevatorToPosCommand(ElevatorSubsystem.level4Position, elevator);
        }
        else {
            return new ElevatorToPosCommand(ElevatorSubsystem.lowPosition, elevator);
        }
    }
}
