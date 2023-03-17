package frc.robot.container4143;

import frc.robot.RobotContainer4143;
import frc.robot.commands4143.PickupOut;
import frc.robot.commands4143.PickupOutRev;
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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.logging.LoggedReceiver;
import frc.lib.logging.Logger;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems4143.Arm;
import frc.robot.subsystems4143.PickupSubsystem;

import java.util.HashMap;
import java.util.List;
import java.util.stream.Stream;

public class AutonomousManager {
    private static final AutonomousOption defaultAuto = AutonomousOption.NOTHING;

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
        PickupSubsystem pickup = container.getPickup();
        initializeNetworkTables();

        // Create an event map for use in all autos
        HashMap<String, Command> eventMap = new HashMap<>();
        autoChooser.setDefaultOption("Nothing", AutonomousOption.NOTHING);
        autoChooser.addOption("loadcone3_pickup1_climb", AutonomousOption.LOADCONE3_PICKUP1_CLIMB);
        autoChooser.addOption("loadcone3_pickup2", AutonomousOption.LOADCONE3_PICKUP2);
        autoChooser.addOption("cornercone1_pickup1_climb", AutonomousOption.CORNERCONE1_PICKUP1_CLIMB);
        autoChooser.addOption("cornercone1_pickup2", AutonomousOption.CORNERCONE1_PICKUP2);
        autoChooser.addOption("cornercube2_pickup1_climb", AutonomousOption.CORNERCUBE2_PICKUP1_CLIMB);
        autoChooser.addOption("loadcube2_pickup1_climb", AutonomousOption.LOADCUBE2_PICKUP1_CLIMB);
        autoChooser.addOption("middleCubeClimb", AutonomousOption.MIDDLE_CUBE_CLIMB);
        autoChooser.addOption("load_Cone4_Climb", AutonomousOption.LOAD_CONE4_CLIMB);
        autoChooser.addOption("load_Cone4_Get_Climb", AutonomousOption.LOAD_CONE4_GET_CLIMB);
        autoChooser.addOption("CORNER_CONE1_CLIMB", AutonomousOption.CORNERCONE1_CLIMB);
        autoChooser.addOption("CORNER_CUBE2_CLIMB", AutonomousOption.CORNERCUBE2_CLIMB);
        autoChooser.addOption("LOAD_CONE3_CLIMB", AutonomousOption.LOADCONE3_CLIMB);
        autoChooser.addOption("LOAD_CUBE2_CLIMB", AutonomousOption.LOADCUBE2_CLIMB);
        autoChooser.addOption("CORNERCUBE2_MOBILITY", AutonomousOption.CORNERCUBE2_MOBILITY);
        autoChooser.addOption("CORNER_CUBE2_CLIMB_V2", AutonomousOption.CORNERCUBE2_CLIMBV2);
        autoChooser.addOption("cornercone3_score1", AutonomousOption.CORNERCONE3_SCORE1);
        SmartDashboard.putData("Autonomous Mode", autoChooser);
        
        eventMap.put("stop", runOnce(swerveDriveSubsystem::stop, swerveDriveSubsystem));
        eventMap.put(
                "shouldClimb",
                either(none(), run(swerveDriveSubsystem::stop, swerveDriveSubsystem), () -> shouldClimb.getBoolean()));
        eventMap.put("levelChargeStation", swerveDriveSubsystem.levelChargeStationCommand());
        eventMap.put("goHigh",
                sequence(
                    arm.setClawClosed(container),
                    Commands.waitSeconds(0.3),
                    arm.setHighPosition()));
        eventMap.put("goHome",
                    arm.setHomePosition());
        eventMap.put("openClaw",
                    arm.setClawOpen(container));
        eventMap.put("PickupOut",
                    new PickupOut(pickup, arm, container));
        eventMap.put("pickOutReverse",
                    new PickupOutRev(pickup, arm, container));
        eventMap.put("pickupCancel",
                    pickup.pickupcancel());
        eventMap.put("rollerCancel",
                    pickup.rollercancel());
        eventMap.put("cubeMode",
                    container.cubeMode());
        eventMap.put("coneMode",
                    container.coneMode());
        eventMap.put(
                    "V2Start",
                    sequence(
                    arm.setClawClosed(container),
                    Commands.waitSeconds(0.3),
                    arm.setHighPosition(),
                    swerveDriveSubsystem.driveForward(1, 0, 0, false, 0.65),
                    runOnce(swerveDriveSubsystem::stop, swerveDriveSubsystem),
                    Commands.waitSeconds(1),
                    arm.setClawOpen(container)));
        autoBuilder = new SwerveAutoBuilder(
                swerveDriveSubsystem::getPose,
                swerveDriveSubsystem::setPose,
                new PIDConstants(3.0, 0.0, 0.0),
                new PIDConstants(4.0, 0.0, 0.001),
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
        CORNERCONE1_PICKUP1_CLIMB(StartingLocation.OPEN, 1, "cornercone1_pickup1_climb", new PathConstraints(2, 2)),
        LOADCONE3_PICKUP1_CLIMB(StartingLocation.OPEN, 1, "loadcone3_pickup1_climb", new PathConstraints(2, 2)),
        LOADCONE3_PICKUP2(StartingLocation.OPEN, 1, "loadcone3_pickup2", new PathConstraints(2.5, 2.5)),
        CORNERCONE1_PICKUP2(StartingLocation.OPEN, 1, "cornercone1_pickup2", new PathConstraints(2, 2)),
        CORNERCUBE2_PICKUP1_CLIMB(StartingLocation.OPEN, 1, "cornercube2_pickup1_climb", new PathConstraints(2, 2)),
        LOADCUBE2_PICKUP1_CLIMB(StartingLocation.OPEN, 1, "loadcube2_pickup1_climb", new PathConstraints(2, 2)),
        NOTHING(StartingLocation.OPEN, 1, "Nothing", new PathConstraints(0.1, 0.1)),
        MIDDLE_CUBE_CLIMB(StartingLocation.OPEN, 1, "middleCubeClimb", new PathConstraints(2, 2)),
        LOAD_CONE4_CLIMB(StartingLocation.OPEN, 1, "load4ConeClimb", new PathConstraints(2, 2)),
        LOAD_CONE4_GET_CLIMB(StartingLocation.OPEN, 1, "load4ConeGetClimb", new PathConstraints(2, 2)),
        CORNERCONE1_CLIMB(StartingLocation.OPEN, 1, "cornercone1_climb", new PathConstraints(2, 2)),
        CORNERCUBE2_CLIMB(StartingLocation.OPEN, 1, "cornercube2_climb", new PathConstraints(2, 2)),
        LOADCONE3_CLIMB(StartingLocation.OPEN, 1, "loadcone3_climb", new PathConstraints(2, 2)),
        LOADCUBE2_CLIMB(StartingLocation.OPEN, 1, "loadcube2_climb", new PathConstraints(2, 2)),
        CORNERCUBE2_MOBILITY(StartingLocation.OPEN, 1, "cornercube2_mobility", new PathConstraints(2, 2)),
        CORNERCUBE2_CLIMBV2(StartingLocation.OPEN, 1, "cornercube2_climbV2", new PathConstraints(2, 2)),
        CORNERCONE3_SCORE1(StartingLocation.OPEN, 1, "cornercone3_score1", new PathConstraints(2, 2))
        ;


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

