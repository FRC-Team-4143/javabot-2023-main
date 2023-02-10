package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.TimesliceRobot;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.controller.Axis;
import frc.lib.controller.CustomXboxController;
import frc.lib.controller.ThrustmasterJoystick;
import frc.lib.logging.Logger;
import frc.lib.loops.UpdateManager;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FieldConstants.PlacementLocation;
import frc.robot.Constants.TimesliceConstants;
import frc.robot.commands.AimAtPoseCommand;
import frc.robot.commands.ClawClose;
import frc.robot.commands.ClawOpen;
import frc.robot.commands.RotateUp;
import frc.robot.commands.RotateDown;
import frc.robot.commands.ElevatorDown;
import frc.robot.commands.ElevatorUp;
import frc.robot.commands.LEDPorpor;
import frc.robot.commands.LEDYel;
import frc.robot.commands.DriveToPositionCommand;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc.robot.subsystems.ArmSubsystem.ArmState;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;



public class RobotContainer {
    // private final ThrustmasterJoystick leftDriveController =
    //         new ThrustmasterJoystick(ControllerConstants.LEFT_DRIVE_CONTROLLER);
    // private final ThrustmasterJoystick rightDriveController =
    //         new ThrustmasterJoystick(ControllerConstants.RIGHT_DRIVE_CONTROLLER);
    // private final LogitechController operatorController =
    //         new LogitechController(ControllerConstants.OPERATOR_CONTROLLER);

            private final CustomXboxController driver = new CustomXboxController(0);
            private final CustomXboxController operator = new CustomXboxController(1);
            private final Axis translationAxis = driver.getLeftYAxis();
    private final Axis strafeAxis = driver.getLeftXAxis();
    private final Axis rotationAxis = driver.getRightXAxis();
    private final SwerveDriveSubsystem swerveDriveSubsystem = new SwerveDriveSubsystem();
    // private final LightsSubsystem lightsSubsystem = new LightsSubsystem();
    private final VisionSubsystem visionSubsystem =
            new VisionSubsystem(swerveDriveSubsystem::addVisionPoseEstimate, swerveDriveSubsystem::getPose);
    //private final ArmSubsystem armSubsystem = new ArmSubsystem();

    private final Arm arm = new Arm();
    private final PickupSubsystem pickupSubsystem = new PickupSubsystem();
    private AddressableLED m_led = new AddressableLED(8);
    private AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(14);


    private AutonomousManager autonomousManager;
    private UpdateManager updateManager;
    
    Trigger driveLT = new Trigger(driver.isLeftTriggerPressed());
    Trigger driveRT = new Trigger(driver.isRightTriggerPressed());

    Trigger operatorLT = new Trigger(operator.isLeftTriggerPressed());
    Trigger operatorRT = new Trigger(operator.isRightTriggerPressed());

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

    public RobotContainer(TimesliceRobot robot) {
        updateManager = new UpdateManager(robot);
        autonomousManager = new AutonomousManager(this);
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();
        // Allocate timeslices
        updateManager.schedule(swerveDriveSubsystem, TimesliceConstants.DRIVETRAIN_PERIOD);
        SmartDashboard.putData("Set Wheel Offset", swerveDriveSubsystem.setWheelOffsets());
        SmartDashboard.putData("Set High Position", arm.setHighPosition());
        SmartDashboard.putData("Set Middle Position", arm.setMidPosition());
        SmartDashboard.putData("Set Home Position", arm.setHomePosition());
        SmartDashboard.putData("0Arm", arm.set0Arm());
        configureBindings();
    }

    private void configureBindings() {
        
        driver.getLeftXAxis().setScale(Constants.SwerveConstants.maxSpeed);
        driver.getLeftYAxis().setScale(Constants.SwerveConstants.maxSpeed);
        driver.getRightXAxis().setScale(Constants.SwerveConstants.maxAngularVelocity);
        driver.getLeftXAxis().setInverted(true);
        driver.getLeftYAxis().setInverted(true);
        driver.getRightXAxis().setInverted(true);
        driver.getButtonB().whileTrue(new ClawOpen(arm));
        driver.nameButtonB("Claw Open");
        driver.getButtonX().whileTrue(new ClawClose(arm));
        driver.nameButtonX("Claw Close");
        driver.getButtonY().whileTrue(new RotateDown(arm));
        driver.nameButtonY("Rotate Down");
        driver.getButtonA().whileTrue(new RotateUp(arm));
        driver.nameButtonA("Rotate Up");
        //driver.getLeftBumper().whileTrue(new ElevatorDown(arm));
        //driver.getRightBumper().whileTrue(new ElevatorUp(arm));
        driveLT.whileTrue(new ElevatorUp(arm));
        driveRT.whileTrue(new ElevatorDown(arm));
        driver.nameLeftTrigger("Elevator Up");
        driver.nameRightTrigger("Elevator Down");
        driver.nameLeftBumper("Elevator Down");
        operator.getLeftBumper().whileTrue(pickupSubsystem.rollIn());
        operator.nameLeftBumper("Rollers In");
        operator.getRightBumper().whileTrue(pickupSubsystem.rollOut());
        operator.nameRightBumper("Rollers Out");
        operator.getButtonY().whileTrue(pickupSubsystem.pickupExtend());
        operator.nameButtonY("extend pickup");
        operator.getButtonA().whileTrue(pickupSubsystem.pickupRetract());
        operator.nameButtonA("Retracts pickup");
        operator.getButtonX().whileTrue(pickupSubsystem.spindexterCW());
        operator.nameButtonX("rotates cone counterclockwise");
        operator.getButtonB().whileTrue(pickupSubsystem.spindexterCCW());
        operator.nameButtonB("rotates cone clockwise");
        operator.getLeftRhombus().whileTrue(new LEDPorpor(m_led, m_ledBuffer));
        operator.nameLeftRhombus("turn lights purple");
        operator.getRightRhombus().whileTrue(new LEDYel(m_led, m_ledBuffer));
        operator.nameRightRhombus("turn lights yellow");
        /* Set default commands */
        // lightsSubsystem.setDefaultCommand(lightsSubsystem.defaultCommand());
        swerveDriveSubsystem.setDefaultCommand(swerveDriveSubsystem.driveCommand(
            getDriveForwardAxis(), getDriveStrafeAxis(), getDriveRotationAxis(), true));
        driver.getRightBumper().toggleOnTrue(swerveDriveSubsystem.driveCommand(
            getDriveForwardAxis(), getDriveStrafeAxis(), getDriveRotationAxis(), false));
        driver.nameRightBumper("Field Centric Toggle");
        /* Set non-button, multi-subsystem triggers */

        /* Set left joystick bindings */
        //driver.getRightRhombus().onTrue(runOnce(swerveDriveSubsystem::zeroRotation, swerveDriveSubsystem));
        //driver
        //        .getLeftRhombus()
        //        .onTrue(runOnce(() -> swerveDriveSubsystem.setPose(new Pose2d()), swerveDriveSubsystem));
       /*  operator
                .getRightRhombus()
                .whileTrue(swerveDriveSubsystem.preciseDriveCommand(
                        getDriveForwardAxis(), getDriveStrafeAxis(), getDriveRotationAxis(), true));*/
        //driver.nameRightRhombus("Reset Gyro Angle");
        //driver.nameLeftRhombus("Reset Pose");
        //operator.nameRightRhombus("Precise Driving");


        // Leveling
        driver.getLeftRhombus().toggleOnTrue(swerveDriveSubsystem.levelChargeStationCommand());
        //operator.getButtonB().whileTrue(run(swerveDriveSubsystem::lock, swerveDriveSubsystem));
        driver.nameLeftRhombus("Level Charge Station");
        //operator.nameButtonB("Lock Wheels");

        /* Set right joystick bindings */
        /*operator.getLeftBumper().whileTrue(swerveDriveSubsystem.characterizeCommand(true, true));
        operator.getRightBumper().whileTrue(swerveDriveSubsystem.characterizeCommand(true, false));
        operator.nameLeftBumper("Characterize Forwards");
        operator.nameRightBumper("Characterize Backwards");*/

        Supplier<Pose2d> targetPoseSupplier = () -> {
            PlacementLocation targetLocation =
                    FieldConstants.getNearestPlacementLocation(swerveDriveSubsystem.getPose());

            var targetPose = targetLocation.robotPlacementPose;

            Logger.log("/SwerveDriveSubsystem/TargetPose", targetPose);
            return targetPose;
        };

        // Supplier<Pose2d> targetAimPoseSupplier = () -> {
        //     PlacementLocation targetLocation =
        //             FieldConstants.getNearestPlacementLocation(swerveDriveSubsystem.getPose());

        //     ArmState armState = armSubsystem.getState();
        //     Pose3d targetPose3d;

        //     switch (armState) {
        //         case HYBRID:
        //             targetPose3d = targetLocation.getHybridPose();
        //             break;
        //         case MID:
        //             targetPose3d = targetLocation.getMidPose();
        //             break;
        //         case HIGH:
        //             targetPose3d = targetLocation.getHighPose();
        //             break;
        //         default:
        //             targetPose3d = new Pose3d(targetLocation.robotPlacementPose.plus(
        //                     new Transform2d(new Translation2d(), Rotation2d.fromDegrees(180))));
        //             break;
        //     }

        //     var targetPose =
        //             targetPose3d.toPose2d().plus(new Transform2d(new Translation2d(), Rotation2d.fromDegrees(180)));

        //     Logger.log("/SwerveDriveSubsystem/TargetPose", targetPose);
        //     return targetPose;
        // };

        //operator.getButtonX().whileTrue(new DriveToPositionCommand(swerveDriveSubsystem, targetPoseSupplier));
        //operator.getButtonY().whileTrue(new AimAtPoseCommand(swerveDriveSubsystem, targetAimPoseSupplier, getDriveForwardAxis(), getDriveStrafeAxis()));
        //operator.nameButtonX("Drive to Pose");
        //operator.nameButtonY("Aim at Pose");

        /* Set operator controller bindings */
        // operatorController.getA().onTrue(runOnce(armSubsystem::setHybrid, armSubsystem));
        // operatorController.getB().onTrue(runOnce(armSubsystem::setMid, armSubsystem));
        // operatorController.getY().onTrue(runOnce(armSubsystem::setHigh, armSubsystem));
        // operatorController.getX().onTrue(runOnce(armSubsystem::setAwaitingDeployment, armSubsystem));
        // operatorController.nameA("Place Hybrid");
        // operatorController.nameB("Place Mid");
        // operatorController.nameY("Place High");
        // operatorController.nameX("Protect Arm");

        driver.sendButtonNamesToNT();
        operator.sendButtonNamesToNT();
        // operatorController.sendButtonNamesToNT();
    }

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

    public void init(){
        arm.setPosition();
    }
}
