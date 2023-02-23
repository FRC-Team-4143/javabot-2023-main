package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import frc.lib.controller.Axis;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.gamePiece;
import frc.robot.Constants.FieldConstants.PlacementLocation;
import frc.lib.logging.Logger;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.commands.AimAtPoseCommand;
import frc.robot.commands.AssistedDriveToPositionCommand;
import frc.robot.commands.DriveToPositionCommand;
import frc.robot.commands4143.*;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems4143.*;
import frc.robot.subsystems4143.SkiSubsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.ArrayList;
import java.util.function.Supplier;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.container4143.CustomXboxController;
import frc.robot.container4143.AutonomousManager;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;

public class RobotContainer4143 {
    private final CustomXboxController driver = new CustomXboxController(0);
    private final CustomXboxController operator = new CustomXboxController(1);
    private final Axis translationAxis = driver.getLeftYAxis();
    private final Axis strafeAxis = driver.getLeftXAxis();
    private final Axis rotationAxis = driver.getRightXAxis();
    public final SwerveDriveSubsystem swerveDriveSubsystem = new SwerveDriveSubsystem();
    private final VisionSubsystem visionSubsystem =
            new VisionSubsystem(swerveDriveSubsystem::addVisionPoseEstimate, swerveDriveSubsystem::getPose);

    private final Arm arm = new Arm();
    private final SkiSubsystem skiSubsystem = new SkiSubsystem();
    private final PickupSubsystem pickupSubsystem = new PickupSubsystem();
    private final FieldPositionSubsystem fieldPositionSubsystem = new FieldPositionSubsystem();

    private AddressableLED m_led = new AddressableLED(8);
    private AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(14);

    public AutonomousManager autonomousManager;
    
    private Trigger driveLT = new Trigger(driver.isLeftTriggerPressed());
    private Trigger driveRT = new Trigger(driver.isRightTriggerPressed());
    private Trigger operatorLT = new Trigger(operator.isLeftTriggerPressed());
    private Trigger operatorRT = new Trigger(operator.isRightTriggerPressed());
    private Trigger driverRSU = new Trigger(() -> {return driver.getRightYAxis().get() > 0.5;});
    private Trigger driverRSD = new Trigger(() -> {return driver.getRightYAxis().get() < -0.5;});
    private Trigger operatorRSU = new Trigger(() -> {return operator.getRightYAxis().get() > 0.5;});
    private Trigger operatorRSD = new Trigger(() -> {return operator.getRightYAxis().get() < -0.5;});

    private boolean blueAlliance;

    public Constants.gamePiece currentMode = Constants.gamePiece.Cube;

    // //Rotator
    // private final JoystickButton rotateUpButton = new JoystickButton(driver, XboxController.Button.kA.value);
    // private final JoystickButton rotateDownButton = new JoystickButton(driver, XboxController.Button.kY.value);

    // //Elevator
    // private final JoystickButton elevatorUpButton = new JoystickButton(driver, XboxController.Button.kStart.value);
    // private final JoystickButton elevatorDownButton = new JoystickButton(driver, XboxController.Button.kBack.value);


    // //pickup 
    // private final JoystickButton rollerinButton = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    // private final JoystickButton rolleroutButton = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    // private final JoystickButton rollerstopButton = new JoystickButton(driver, XboxController.Button.kRightStick.value);

    // private final JoystickButton pickupInButton = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
    // private final JoystickButton pickupOutButton = new JoystickButton(operator, XboxController.Button.kRightBumper.value);

    public RobotContainer4143(TimedRobot robot) {
        System.out.println("start of robotcontainer4143");
        autonomousManager = new AutonomousManager(this);
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();
    
        configureBindings();
        arm.setPosition();
        blueAlliance = DriverStation.getAlliance() == Alliance.Blue;
    }

    private void configureBindings() {
        SmartDashboard.putData("Set Wheel Offset", swerveDriveSubsystem.setWheelOffsets());
        SmartDashboard.putData("Set High Position", arm.setHighPosition());
        SmartDashboard.putData("Set Middle Position", arm.setMidPosition());
        SmartDashboard.putData("Set Home Position", arm.setHomePosition());
        SmartDashboard.putData("0Arm", arm.set0Arm());


        Supplier<Pose2d> testPoseSupplier = () -> {
            var targetPose = new Pose2d(1, 1, new Rotation2d(0));
            return targetPose;
        };
        SmartDashboard.putData("Drive to default position", new DriveToPositionCommand(swerveDriveSubsystem, testPoseSupplier));
        
        Supplier<Pose2d> targetPoseSupplier = () -> {
            PlacementLocation targetLocation =
                    FieldConstants.getNearestPlacementLocation(swerveDriveSubsystem.getPose());

            var targetPose = targetLocation.robotPlacementPose;

            //Logger.log("/SwerveDriveSubsystem/TargetPose", targetPose);
            return targetPose;
        };
    
        //Joystick inputs, test operator joystick
        driver.getLeftXAxis().setScale(Constants.SwerveConstants.maxSpeed);
        driver.getLeftYAxis().setScale(Constants.SwerveConstants.maxSpeed);
        driver.getRightXAxis().setScale(Constants.SwerveConstants.maxAngularVelocity * .5);
        driver.getLeftXAxis().setInverted(true);
        driver.getLeftYAxis().setInverted(true);
        driver.getRightXAxis().setInverted(true);
        operator.getLeftXAxis().setScale(Constants.SwerveConstants.maxSpeed);
        operator.getLeftYAxis().setScale(Constants.SwerveConstants.maxSpeed);
        operator.getRightXAxis().setScale(Constants.SwerveConstants.maxAngularVelocity * .5);
        operator.getLeftXAxis().setInverted(true);
        operator.getLeftYAxis().setInverted(true);
        operator.getRightXAxis().setInverted(true);
        
        //Joystick press buttons
        driver.getLeftThumb().whileTrue(run(swerveDriveSubsystem::lock, swerveDriveSubsystem));
        driver.nameLeftThumb("Lock Wheels");
        driver.getRightThumb().toggleOnTrue(swerveDriveSubsystem.driveCommand(
            getDriveForwardAxis(), getDriveStrafeAxis(), getDriveRotationAxis(), false, driver.getLeftBumper()));
        driver.nameRightThumb("Field Centric Toggle");
        operator.getLeftThumb().whileTrue(run(swerveDriveSubsystem::lock, swerveDriveSubsystem));
        operator.nameLeftThumb("Lock Wheels");
        operator.getRightThumb().toggleOnTrue(swerveDriveSubsystem.driveCommand(
            getDriveForwardAxis(), getDriveStrafeAxis(), getDriveRotationAxis(), false, driver.getLeftBumper()));
        operator.nameRightThumb("Field Centric Toggle");
        
        //Rhombus buttons
        driver.getLeftRhombus().toggleOnTrue(swerveDriveSubsystem.levelChargeStationCommand());
        driver.nameLeftRhombus("Level Charge Station");
        driver.getRightRhombus().onTrue(runOnce(() -> {
            if(currentMode == Constants.gamePiece.Cube){currentMode = gamePiece.Cone;}
                else{currentMode = gamePiece.Cube;}
            }).ignoringDisable(true));
        driver.nameRightRhombus("Cone/cube mode");
        
        operator.getLeftRhombus().toggleOnTrue(swerveDriveSubsystem.levelChargeStationCommand());
        operator.nameLeftRhombus("Level Charge Station");
        operator.getRightRhombus().onTrue(runOnce(() -> {
            if(currentMode == Constants.gamePiece.Cube){currentMode = gamePiece.Cone;}
                else{currentMode = gamePiece.Cube;}
            }).ignoringDisable(true));
        operator.nameRightRhombus("Cone/cube mode");


        //Trigger buttons
        driveLT.whileTrue(new PickupOutRev(pickupSubsystem,arm, this));
        driveRT.whileTrue(new PickupOut(pickupSubsystem,arm, this));
        operatorLT.whileTrue(new PickupOutRev(pickupSubsystem,arm,this));
        operatorRT.whileTrue(new PickupOut(pickupSubsystem,arm,this));

        //driverRSU.whileTrue(skiSubsystem.setSkiUp());
        //driverRSD.whileTrue(skiSubsystem.setSkiDown());
        operatorRSU.whileTrue(pickupSubsystem.spindexterCW());
        operatorRSD.whileTrue(pickupSubsystem.spindexterCCW());
        driverRSU.whileTrue(pickupSubsystem.spindexterCW());
        driverRSD.whileTrue(pickupSubsystem.spindexterCCW());

        //Bumper buttons
        driver.getRightBumper().onTrue(new ClawToggle(arm, this));
        operator.getRightBumper().onTrue(new ClawToggle(arm, this));

        // X, Y, A, and B buttons
        driver.getButtonY().whileTrue(new ElevatorUp(arm));
        driver.nameButtonY("Elevator Up");
        driver.getButtonA().whileTrue(new ElevatorDown(arm));
        driver.nameButtonA("Elevator Down");
        driver.getButtonX().whileTrue(new RotateUp(arm));
        driver.nameButtonX("Rotate Up");
        driver.getButtonB().whileTrue(new RotateDown(arm));
        driver.nameButtonB("Rotate Down");
        operator.getButtonY().whileTrue(new ElevatorUp(arm));
        operator.nameButtonY("Elevator Up");
        operator.getButtonA().whileTrue(new ElevatorDown(arm));
        operator.nameButtonA("Elevator Down");
        operator.getButtonX().whileTrue(new RotateUp(arm));
        operator.nameButtonX("Rotate Up");
        operator.getButtonB().whileTrue(new RotateDown(arm));
        operator.nameButtonB("Rotate Down");

        //Dpad buttons
        driver.getDPadDown().onTrue(arm.setHomePosition());
        driver.getDPadRight().onTrue(arm.setMidPosition());
        //driver.getDPadLeft().onTrue(arm.setHybridPosition);
        driver.getDPadUp().onTrue(arm.setHighPosition());
        operator.getDPadDown().onTrue(arm.setHomePosition());
        operator.getDPadRight().onTrue(arm.setMidPosition());
        //operator.getDPadLeft().onTrue(arm.setHybridPosition);
        operator.getDPadUp().onTrue(arm.setHighPosition());

        //Autodrive stuff
        //driver.getDPadLeft().whileTrue(new ProxyCommand(()->autonomousManager.autoBuilder.followPathGroup(pathGroupOnTheFly())));
        //driver.getDPadDown().whileTrue(new DriveToPositionCommand(swerveDriveSubsystem, targetPoseSupplier));
        //driver.getDPadLeft().whileTrue(new AimAtPoseCommand(
        //    swerveDriveSubsystem, targetAimPoseSupplier, getDriveForwardAxis(), getDriveStrafeAxis()));
        //driver.getDPadUp().whileTrue(new AssistedDriveToPositionCommand(
        //    swerveDriveSubsystem, targetPoseSupplier, getDriveForwardAxis()));
        //driver.getDPadRight().whileTrue(new DriveToPositionCommand(swerveDriveSubsystem, smartDashPoseSupplier));
        //driver.getDPadRight().whileTrue(new ProxyCommand(()->autonomousManager.autoBuilder.followPathGroup(pathGroupOnTheFlyscore())));

        //Controls that we haven't made commands for
        //driver.getLeftBumper().whileTrue(pickupSubsystem.rollIn()); //Change to auto drive
        //driver.nameLeftBumper("Rollers In");
        //driver.nameRightBumper("Rollers Out");
        

        //Old buttons to keep
        //driveLT.whileTrue(new ElevatorUp(arm));
        //driveRT.whileTrue(new ElevatorDown(arm));
        //driver.nameLeftTrigger("Pick up out");
        //driver.nameRightTrigger("Elevator Down");
        // operator.getButtonY().whileTrue(pickupSubsystem.pickupExtend());
        // operator.nameButtonY("extend pickup");
        // operator.getButtonA().whileTrue(pickupSubsystem.pickupRetract());
        // operator.nameButtonA("Retracts pickup");
        // operator.getButtonX().whileTrue(pickupSubsystem.spindexterCW());
        // operator.nameButtonX("rotates cone counterclockwise");
        // operator.getButtonB().whileTrue(pickupSubsystem.spindexterCCW());
        // operator.nameButtonB("rotates cone clockwise");
        // operator.getLeftRhombus().whileTrue(new LEDPorpor(m_led, m_ledBuffer));
        // operator.nameLeftRhombus("turn lights purple");


        fieldPositionSubsystem.setDefaultCommand(fieldPositionSubsystem.scoreSelectCommand(operator.getLeftYAxis(), operator.getLeftXAxis()));
        

        swerveDriveSubsystem.setDefaultCommand(swerveDriveSubsystem.driveCommand(
            getDriveForwardAxis(), getDriveStrafeAxis(), getDriveRotationAxis(), true, driver.getLeftBumper()));

        driver.sendButtonNamesToNT();
        operator.sendButtonNamesToNT();
        
        
        /* Set non-button, multi-subsystem triggers */

        /* Set left joystick bindings */
        //driver
        //        .getLeftRhombus()
        //        .onTrue(runOnce(() -> swerveDriveSubsystem.setPose(new Pose2d()), swerveDriveSubsystem));
       /*  operator
                .getRightRhombus()swz
                .whileTrue(swerveDriveSubsystem.preciseDriveCommand(
                        getDriveForwardAxis(), getDriveStrafeAxis(), getDriveRotationAxis(), true));*/
        
        //driver.nameLeftRhombus("Reset Pose");
        //operator.nameRightRhombus("Precise Driving");

        
        Supplier<Pose2d> smartDashPoseSupplier = () -> {
            PlacementLocation targetLocation =
                FieldConstants.placingPoses[fieldPositionSubsystem.getXPosition()-1];

            var targetPose = targetLocation.robotPlacementPose;

            //Logger.log("/SwerveDriveSubsystem/TargetPose", targetPose);
            return targetPose;
        };

        Supplier<Pose2d> targetAimPoseSupplier = () -> {
            PlacementLocation targetLocation =
                    FieldConstants.getNearestPlacementLocation(swerveDriveSubsystem.getPose());

            //ArmState armState = armSubsystem.getState();
            Pose3d targetPose3d;

            //switch (armState) {
            //    case HYBRID:
            //        targetPose3d = targetLocation.getHybridPose();
            //       break;
            //    case MID:
            //        targetPose3d = targetLocation.getMidPose();
            //       break;
            //    case HIGH:
                    targetPose3d = targetLocation.getHighPose();
            //       break;
            //    default:
            //        targetPose3d = new Pose3d(targetLocation.robotPlacementPose.plus(
            //                new Transform2d(new Translation2d(), Rotation2d.fromDegrees(180))));
            //        break;
            //}

            var targetPose =
                    targetPose3d.toPose2d().plus(new Transform2d(new Translation2d(), Rotation2d.fromDegrees(0)));
            Logger.log("/SwerveDriveSubsystem/TargetPose", targetPose);
            return targetPose;
        };

        
        
    }


    public ArrayList<PathPlannerTrajectory> pathGroupOnTheFly(){
        Translation2d vector;
        if (blueAlliance) {
            vector = (new Translation2d(12.6, 6.25).minus(swerveDriveSubsystem.getPose().getTranslation()));}
        else {
            vector = (new Translation2d(12.6, FieldConstants.fieldLength - 6.25).minus(swerveDriveSubsystem.getPose().getTranslation()));}
        // System.out.println(Math.atan2(vector.getX(),vector.getY()));
        // System.out.println(vector.getAngle().minus(new Rotation2d(0)));
        // System.out.println("Path group on the fly is being called");
        ArrayList<PathPlannerTrajectory> PGOTF = new ArrayList<PathPlannerTrajectory>();
        if (blueAlliance) {
        PGOTF.add(0,
        PathPlanner.generatePath(
          new PathConstraints(4, 2), 
          new PathPoint(swerveDriveSubsystem.getPose().getTranslation(),vector.getAngle(), swerveDriveSubsystem.getPose().getRotation()),
          new PathPoint(new Translation2d(12.6, 6.25), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0), 3), // position, heading(direction of travel), holonomic rotation, velocity override
          new PathPoint(new Translation2d(15.25, 6.25), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)) // position, heading(direction of travel), holonomic rotation
          
          ));
        }
        else {
            PathPlanner.generatePath(
          new PathConstraints(4, 2), 
          new PathPoint(swerveDriveSubsystem.getPose().getTranslation(),vector.getAngle(), swerveDriveSubsystem.getPose().getRotation()),
          new PathPoint(new Translation2d(12.6, FieldConstants.fieldLength - 6.25), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0), 3), // position, heading(direction of travel), holonomic rotation, velocity override
          new PathPoint(new Translation2d(15.25, FieldConstants.fieldLength - 6.25), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)) // position, heading(direction of travel), holonomic rotation
          
          );
        }
        return PGOTF; 
        
      }

      public ArrayList<PathPlannerTrajectory> pathGroupOnTheFlyscore(){
            Translation2d vector;
            if (blueAlliance) {
                vector = (new Translation2d(5.80, FieldConstants.fieldLength - 4.68).minus(swerveDriveSubsystem.getPose().getTranslation()));}
            else {
                vector = (new Translation2d(5.80, FieldConstants.fieldLength - 4.68).minus(swerveDriveSubsystem.getPose().getTranslation()));}
            System.out.println(Math.atan2(vector.getX(),vector.getY()));
            System.out.println(vector.getAngle().minus(new Rotation2d(0)));
            System.out.println("Path group on the fly is being called");
            ArrayList<PathPlannerTrajectory> PGOTF = new ArrayList<PathPlannerTrajectory>();
            if (blueAlliance) {
                PGOTF.add(0,
            PathPlanner.generatePath(
              new PathConstraints(4, 2), 
              new PathPoint(swerveDriveSubsystem.getPose().getTranslation(),vector.getAngle(), swerveDriveSubsystem.getPose().getRotation()), 
              new PathPoint(new Translation2d(5.80, 5), Rotation2d.fromDegrees(-177.99), Rotation2d.fromDegrees(-179),3), // position, heading(direction of travel), holonomic rotation, velocity override
              new PathPoint(new Translation2d(2.3, 4.7), Rotation2d.fromDegrees(-90), Rotation2d.fromDegrees(-179)) // position, heading(direction of travel), holonomic rotation
              ));
            }
            else {
                PGOTF.add(0,
            PathPlanner.generatePath(
              new PathConstraints(4, 2), 
              new PathPoint(swerveDriveSubsystem.getPose().getTranslation(),vector.getAngle(), swerveDriveSubsystem.getPose().getRotation()), 
              new PathPoint(new Translation2d(5.80, FieldConstants.fieldLength - 5), Rotation2d.fromDegrees(-177.99), Rotation2d.fromDegrees(-179),3), // position, heading(direction of travel), holonomic rotation, velocity override
              new PathPoint(new Translation2d(2.3, FieldConstants.fieldLength - 4.7), Rotation2d.fromDegrees(-90), Rotation2d.fromDegrees(-179)) // position, heading(direction of travel), holonomic rotation
              ));
            }
            return PGOTF; 
          } 
          
    
    // square around charge station
    // public ArrayList<PathPlannerTrajectory> pathGroupOnTheFly(){
    //     Translation2d vector = (new Translation2d(5.80, 4.68).minus(swerveDriveSubsystem.getPose().getTranslation()));
    //     System.out.println(Math.atan2(vector.getX(),vector.getY()));
    //     System.out.println(vector.getAngle().minus(new Rotation2d(0)));
    //     System.out.println("Path group on the fly is being called");
    //     ArrayList<PathPlannerTrajectory> PGOTF = new ArrayList<PathPlannerTrajectory>();
    //     PGOTF.add(0,
    //     PathPlanner.generatePath(
    //       new PathConstraints(3, 2), 
    //       new PathPoint(swerveDriveSubsystem.getPose().getTranslation(),vector.getAngle(), swerveDriveSubsystem.getPose().getRotation()), 
    //       new PathPoint(new Translation2d(5.80, 4.68), Rotation2d.fromDegrees(-177.99), Rotation2d.fromDegrees(-179)), // position, heading(direction of travel), holonomic rotation, velocity override
    //       new PathPoint(new Translation2d(2.3, 4.64), Rotation2d.fromDegrees(-90), Rotation2d.fromDegrees(-179)), // position, heading(direction of travel), holonomic rotation
    //       new PathPoint(new Translation2d(2.3, 1.2), Rotation2d.fromDegrees(-90), Rotation2d.fromDegrees(-179)),
    //       new PathPoint(new Translation2d(5.70, 1.2), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(-179)),
    //       new PathPoint(new Translation2d(5.70, 3.61), Rotation2d.fromDegrees(90), Rotation2d.fromDegrees(-179))
    //       ));
    //     return PGOTF; 
    //   }
    
    public Command getAutonomousCommand() {
        return autonomousManager.getAutonomousCommand();
    }

    public Axis getDriveForwardAxis() {
        return translationAxis;
    }

    public Axis getDriveStrafeAxis() {
        return strafeAxis;
    }

    public Axis getDriveRotationAxis() {
        return rotationAxis;
    }

    public SwerveDriveSubsystem getSwerveDriveSubsystem() {
        return swerveDriveSubsystem;
    }

    public Arm getArm(){
        return arm;
    }

    // public LightsSubsystem getLightsSubsystem() {
    //     return lightsSubsystem;
    // }

    public VisionSubsystem getVisionSubsystem() {
        return visionSubsystem;
    }

    // public ArmSubsystem getArmSubsystem() {
    //     return armSubsystem;
    // }
}