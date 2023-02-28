package frc.robot.container4143;

import frc.robot.RobotContainer4143;
import frc.robot.Constants;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.logging.LoggedReceiver;
import frc.lib.logging.Logger;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems4143.Arm;

import java.util.HashMap;
import java.util.List;
import java.util.stream.Stream;

public class AutonomousManager {
    private static final AutonomousOption defaultAuto = AutonomousOption.PLACE1ANDCLIMB;

    // Add tunables for all autonomous configuration options
    LoggedReceiver waitDuration;
    LoggedReceiver startPosition;
    LoggedReceiver gamePieces;
    LoggedReceiver shouldClimb;

    private String previousStartPosition = defaultAuto.startPosition.name();
    private int previousGamePieces = defaultAuto.gamePieces;

    public SwerveAutoBuilder autoBuilder;

    private List<PathPlannerTrajectory> chosenAuto = defaultAuto.getPath();
    private static final SendableChooser<AutonomousOption> autoChooser = new SendableChooser<>();

    SwerveDriveSubsystem swerveDriveSubsystem;

    public AutonomousManager(RobotContainer4143 container) {
        swerveDriveSubsystem = container.getSwerveDriveSubsystem();
        Arm arm = container.getArm();
        initializeNetworkTables();

        // Create an event map for use in all autos
        HashMap<String, Command> eventMap = new HashMap<>();
        autoChooser.setDefaultOption("PLACE1ANDCLIMB", AutonomousOption.PLACE1ANDCLIMB);
        autoChooser.addOption("CORNERPLACE1ANDCLIMB", AutonomousOption.CORNERPLACE1ANDCLIMB);
        SmartDashboard.putData("Autonomous Mode", autoChooser);
        eventMap.put("stop", runOnce(swerveDriveSubsystem::stop, swerveDriveSubsystem));
        eventMap.put(
                "shouldClimb",
                either(none(), run(swerveDriveSubsystem::stop, swerveDriveSubsystem), () -> shouldClimb.getBoolean()));
        eventMap.put("levelChargeStation", swerveDriveSubsystem.levelChargeStationCommand());
        eventMap.put("goHigh",
                sequence(
                    arm.setClawClosed(container),
                    arm.setHighPosition()));
        eventMap.put("placeHigh",
                    sequence(
                        arm.setClawOpen(),
                        arm.setHomePosition()));


        autoBuilder = new SwerveAutoBuilder(
                swerveDriveSubsystem::getPose,
                swerveDriveSubsystem::setPose,
                new PIDConstants(3.0, 0.0, 0.0),
                new PIDConstants(1.0, 0.0, 0.001),
                (ChassisSpeeds velocity) -> swerveDriveSubsystem.setVelocity(velocity, false, false),
                eventMap,
                true,
                swerveDriveSubsystem);

        // Prevent the server from running at competitions
        if (!Constants.competitionMode) {
            PathPlannerServer.startServer(5811);
        }
    }

    public void update() {
        var newStartPosition = startPosition.getString();
        var newGamePieces = gamePieces.getInteger();

        // Only update the chosen auto if a different option has been chosen
        if (previousStartPosition != newStartPosition || previousGamePieces != newGamePieces) {
            // Match the auto based on the dashboard configuration
            List<AutonomousOption> options = Stream.of(AutonomousOption.values())
                    .filter(option ->
                            option.startPosition.name() == newStartPosition && option.gamePieces == newGamePieces)
                    .toList();

            if (options.size() == 1) chosenAuto = options.get(0).getPath();
            else chosenAuto = defaultAuto.getPath();

            // Determine all of the game piece options for this starting position
            long[] gamePieceOptions = Stream.of(AutonomousOption.values())
                    .filter(option -> option.startPosition.name().equals(newStartPosition))
                    .mapToLong(option -> option.gamePieces)
                    .toArray();

            Logger.log("/Autonomous/Game Piece Options", gamePieceOptions);

            previousStartPosition = newStartPosition;
            previousGamePieces = (int) newGamePieces;
        }
    }

    public Command getAutonomousCommand() {

        AutonomousOption nameOfSelectedAuto = autoChooser.getSelected();
        Command autonomousCommand;

        System.out.println(nameOfSelectedAuto);
        System.out.println(nameOfSelectedAuto.pathName);

        // Run the default auto if an invalid auto has been chosen
        //try {
            nameOfSelectedAuto.getPath();
            autonomousCommand = nameOfSelectedAuto.getCommand(autoBuilder);
        //} catch (Exception e) {
        //    System.out.println("Running default auto");
        //    autonomousCommand = AutonomousOption.valueOf(defaultAuto.name()).getCommand(autoBuilder);
        //}

        // Return an empty command group if no auto is specified
        return autonomousCommand;

        // Command chosenPathCommand = autoBuilder.fullAuto(chosenAuto);
        // var chosenWaitDuration = waitDuration.getInteger();
        // if (chosenWaitDuration > 0) chosenPathCommand.beforeStarting(waitSeconds(chosenWaitDuration));
        // return chosenPathCommand;
    }

    private void initializeNetworkTables() {
        waitDuration = Logger.tunable("/Autonomous/Wait Duration", 0.0);
        startPosition = Logger.tunable(
                "/Autonomous/Start Position", defaultAuto.startPosition.name()); // 0 = Left, 1 = Center, 2 = Right
        gamePieces = Logger.tunable("/Autonomous/Game Pieces", defaultAuto.gamePieces);
        shouldClimb = Logger.tunable("/Autonomous/Should Climb", true);

        Logger.log("/Autonomous/Start Position Options", getStartingLocations());

        // Determine all of the game piece options for this starting position
        long[] gamePieceOptions = Stream.of(AutonomousOption.values())
                .filter(option -> option.startPosition.equals(defaultAuto.startPosition))
                .mapToLong(option -> option.gamePieces)
                .toArray();

        Logger.log("/Autonomous/Game Piece Options", gamePieceOptions);
    }

    private enum AutonomousOption {
        PLACE1ANDCLIMB(StartingLocation.OPEN, 1, "place1andclimb", new PathConstraints(4, 4)),
        CORNERPLACE1ANDCLIMB(StartingLocation.OPEN, 1, "cornerplace1andclimb", new PathConstraints(4, 4)),
        PLACE2ANDCLIMB(StartingLocation.OPEN, 2, "place2andclimb", new PathConstraints(5, 4)),
        PLACE3ANDCLIMB(StartingLocation.OPEN, 3, "place3andclimb", new PathConstraints(6, 5)),
        FIVEPIECE(StartingLocation.OPEN, 5, "fivepiece", new PathConstraints(5, 6));

        private List<PathPlannerTrajectory> path;
        public String pathName;
        private PathConstraints constraints;
        public StartingLocation startPosition;
        public int gamePieces;

        private AutonomousOption(
                StartingLocation startPosition, int gamePieces, String pathName, PathConstraints constraints) {
            this.startPosition = startPosition;
            this.gamePieces = gamePieces;
            this.pathName = pathName;
            this.constraints = constraints;
        }

        public List<PathPlannerTrajectory> getPath() {
            // Lazy load the path
            if (path == null) path = PathPlanner.loadPathGroup(pathName, constraints);

            return path;
        }

        public Command getCommand(SwerveAutoBuilder autoBuilder) {
            return autoBuilder.fullAuto(path);
        }
    }

    private enum StartingLocation {
        OPEN,
        STATION,
        CABLE
    }

    public static String[] getStartingLocations() {
        return Stream.of(StartingLocation.values()).map(StartingLocation::name).toArray(String[]::new);
    }

    public static String[] getAutonomousOptionNames() {
        return Stream.of(AutonomousOption.values()).map(AutonomousOption::name).toArray(String[]::new);
    }
}

