package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.controller.Axis;
import frc.lib.gyro.GenericGyro;
import frc.lib.gyro.NavXGyro;
import frc.lib.gyro.PigeonGyro;
import frc.lib.interpolation.MovingAverageVelocity;
import frc.lib.logging.LoggedReceiver;
import frc.lib.logging.Logger;
import frc.lib.loops.Updatable;
import frc.lib.math.MathUtils;
import frc.lib.swerve.SwerveDriveSignal;
//import frc.lib.swerve.SwerveModule;  //4143
import frc.robot.subsystems4143.SwerveModule;  //4143
import frc.robot.Constants;
import frc.robot.Constants.GlobalConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.commands.FeedForwardCharacterization.FeedForwardCharacterizationData;
import java.util.function.Consumer;
import java.util.function.Supplier;
import java.util.stream.DoubleStream;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d; //4143
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; //4143

public class SwerveDriveSubsystem extends SubsystemBase implements Updatable {
    private final SwerveDrivePoseEstimator swervePoseEstimator;
    private final Field2d m_field = new Field2d(); //4143
    private boolean dpadR;
    private boolean isClimbingForward = false;
    private boolean isClimbingBackwards = false;
    private Pose2d pose = new Pose2d();
    private final MovingAverageVelocity velocityEstimator = new MovingAverageVelocity(3);
    private ChassisSpeeds velocity = new ChassisSpeeds();
    private SwerveDriveSignal driveSignal = new SwerveDriveSignal();

    private SwerveModule[] modules;

    private GenericGyro gyro;

    boolean isCharacterizing = false;

    private LoggedReceiver isSecondOrder;

    public SwerveDriveSubsystem() {
        if (SwerveConstants.hasPigeon)
            gyro = new PigeonGyro(SwerveConstants.PIGEON_PORT, "CANivore");
        else gyro = new NavXGyro();

        modules = new SwerveModule[] {
            new SwerveModule(0, Constants.SwerveConstants.Mod0.constants),
            new SwerveModule(1, Constants.SwerveConstants.Mod1.constants),
            new SwerveModule(2, Constants.SwerveConstants.Mod2.constants),
            new SwerveModule(3, Constants.SwerveConstants.Mod3.constants)
        };

        Timer.delay(.1);
        resetToAbsolute();

        // Initialize the swerve drive pose estimator with access to the module positions.
        swervePoseEstimator = new SwerveDrivePoseEstimator(
                SwerveConstants.swerveKinematics,
                getGyroRotation(),
                getModulePositions(),
                new Pose2d(),
                VecBuilder.fill(0.01, 0.01, 0.01),
                VecBuilder.fill(0.8, 0.8, 0.8));

        // Allow us to toggle on second order kinematics
        isSecondOrder = Logger.tunable("/SwerveDriveSubsystem/isSecondOrder", false);

        SmartDashboard.putData("Field", m_field); //4143
    }

    public void resetToAbsolute() {
            // Reset each module using its absolute encoder to avoid having modules fail to align
            for (SwerveModule module : modules) {
                module.resetToAbsolute();
            }
    }

    //4143
    public CommandBase setWheelOffsets() {
        return runOnce(() -> {
            for (SwerveModule module : modules) {
                module.setWheelOffsets();
            }
            System.out.println("Set wheel offset has been enabled");
        }).ignoringDisable(true);
    }

    public Command driveCommand(Axis forward, Axis strafe, Axis rotation, boolean isFieldOriented, Trigger dpadR) {
        return run(() -> {
            setVelocity(new ChassisSpeeds(forward.get(true), strafe.get(true), rotation.get(true)), isFieldOriented);
            this.dpadR = dpadR.getAsBoolean();
           });
    }
    public Command driveForward(double forward, double strafe, double rotation, boolean isFieldOriented) {
        return run(() -> {
            setVelocity(new ChassisSpeeds(forward, strafe, rotation), isFieldOriented);
           });
    }
    public Command preciseDriveCommand(Axis forward, Axis strafe, Axis rotation, boolean isFieldOriented) {
        var speedMultiplier = SwerveConstants.preciseDrivingModeSpeedMultiplier;

        return run(() -> {
            setVelocity(
                    new ChassisSpeeds(
                            speedMultiplier * forward.get(true),
                            speedMultiplier * strafe.get(true),
                            speedMultiplier * rotation.get(true)),
                    isFieldOriented);
        });
    }

    public Command levelChargeStationCommand() {
        var constraints = new TrapezoidProfile.Constraints(0.6, 0.6);
        var tiltController = new ProfiledPIDController(0.06, 0, 0, constraints);
        
        // End with no pitch and stationary
        State goal = new State(0, 0);


        // Four degrees of tolerance
        tiltController.setTolerance(4, 0.1);

        return run(() -> {
                    double pitch = getTiltAmount();
                    // SmartDashboard.putNumber("Pitch", pitch);
                    double rampRate = gyro.getRotationRates3d().getY();
                 //   SmartDashboard.putBoolean("Is Climbing Forward", isClimbingForward);
                   // SmartDashboard.putBoolean("Is Climbing Backwards", isClimbingBackwards);
                    
                    // Negative pitch -> drive forward, Positive pitch -> drive backward
                    double gyroPitch = gyro.getRotation3d().getY() * 180 / Math.PI;
                    SmartDashboard.putNumber("Gyro Pitch", gyroPitch);
                    Translation2d direction = new Translation2d(
                            getNormalVector3d().getX(), getNormalVector3d().getY());
                    if(gyroPitch > 12){isClimbingForward = true; isClimbingBackwards = false;}
                    if(gyroPitch < -12){isClimbingBackwards = true; isClimbingForward = false;}
                    if(pitch < 4){isClimbingBackwards = false; isClimbingForward = false;}
                    
                    Translation2d finalDirection = direction.times(tiltController.calculate(pitch, goal));
                    ChassisSpeeds backwardsVelocity = new ChassisSpeeds(-finalDirection.getX(), -finalDirection.getY(), 0);
                    ChassisSpeeds velocity = new ChassisSpeeds(finalDirection.getX(), finalDirection.getY(), 0);
                    //This locks the wheels within 2 degrees
                    if (MathUtils.equalsWithinError(pitch, 0, 3)) lock();
                    //Trying to backup when it goes overboard
                  
                    // else if(isClimbingForward && gyroPitch < 12){setVelocity(backwardsVelocity, false, false) ;}
                    // else if(isClimbingBackwards && gyroPitch > -12){setVelocity(backwardsVelocity, false, false);}

                    //drive forward
                    else setVelocity(velocity, false,false);
                })
                .repeatedly();
    }

    public Command characterizeCommand(boolean forwards, boolean isDriveMotors) {
        Consumer<Double> voltageConsumer = isDriveMotors
                ? (Double voltage) -> {
                    for (SwerveModule module : modules) {
                        module.setDriveCharacterizationVoltage(voltage);
                    }
                }
                : (Double voltage) -> {
                    for (SwerveModule module : modules) {
                        module.setAngleCharacterizationVoltage(voltage);
                    }
                };

        Supplier<Double> velocitySupplier = isDriveMotors
                ? () -> {
                    return DoubleStream.of(
                                    modules[0].getState().speedMetersPerSecond,
                                    modules[1].getState().speedMetersPerSecond,
                                    modules[2].getState().speedMetersPerSecond,
                                    modules[3].getState().speedMetersPerSecond)
                            .average()
                            .getAsDouble();
                }
                : () -> {
                    return DoubleStream.of(
                                    modules[0].getAngularVelocity(),
                                    modules[1].getAngularVelocity(),
                                    modules[2].getAngularVelocity(),
                                    modules[3].getAngularVelocity())
                            .average()
                            .getAsDouble();
                };

        return new FeedForwardCharacterization(
                        this,
                        forwards,
                        new FeedForwardCharacterizationData("Swerve Drive"),
                        voltageConsumer,
                        velocitySupplier)
                .beforeStarting(() -> isCharacterizing = true)
                .finallyDo((boolean interrupted) -> isCharacterizing = false);
    }

    public void switchToBackupGyro() {
        gyro = new NavXGyro();
    }

    public Pose2d getPose() {
        return pose;
    }

    public void setPose(Pose2d pose) {
        this.pose = pose;
        swervePoseEstimator.resetPosition(getGyroRotation(), getModulePositions(), pose);
    }

    public void addVisionPoseEstimate(Pose2d visionPose, double timestamp) { //4143

        double dist = visionPose.getTranslation().getDistance(pose.getTranslation());
        Transform2d vector = pose.minus(visionPose);
        Logger.log("/SwerveDriveSubsystem/VisionError", dist);
        if(/*dpadR && */ visionPose.getY() > 0 + 0.38 && 
        !(visionPose.getY() > 4 && visionPose.getY() < 4.02) && 
        visionPose.getY() < 7.9248 - 0.38 && 
        !(visionPose.getX() > 8.25 && visionPose.getX() < 8.29) 
        && visionPose.getX() > 0 && visionPose.getX() < 16.4592) 
        { 
             //if(dist < 1.1) {
                swervePoseEstimator.addVisionMeasurement(visionPose, timestamp);
             //} else {
             //   swervePoseEstimator.addVisionMeasurement(pose.transformBy(vector.div(-dist)), timestamp);
            //}
        }
    }

    /**
     * @return The robot relative velocity of the drivetrain
     */
    public ChassisSpeeds getVelocity() {
        return velocity;
    }

    /**
     * @return The potentially field relative desired velocity of the drivetrain
     */
    public ChassisSpeeds getDesiredVelocity() {
        return (ChassisSpeeds) driveSignal;
    }

    public double getVelocityMagnitude() {
        return Math.sqrt(Math.pow(velocity.vxMetersPerSecond, 2) + Math.pow(velocity.vyMetersPerSecond, 2));
    }

    public Rotation2d getVelocityRotation() {
        return (new Translation2d(velocity.vxMetersPerSecond, velocity.vxMetersPerSecond)).getAngle();
    }

    public ChassisSpeeds getSmoothedVelocity() {
        return velocityEstimator.getAverage();
    }

    public Rotation2d getGyroRotation() {
        return gyro.getRotation2d();
    }

    public Rotation2d getRotation() {
        return pose.getRotation();
    }

    public Rotation3d getGyroRotation3d() {
        return gyro.getRotation3d();
    }

    public Translation3d getNormalVector3d() {
        return new Translation3d(0, 0, 1).rotateBy(getGyroRotation3d());
    }

    /**is in degrees*/
    public double getTiltAmount() {
        return Math.toDegrees(Math.acos(getNormalVector3d().getZ()));
    }

    public Rotation2d getTiltDirection() {
        return new Rotation2d(getNormalVector3d().getX(), getNormalVector3d().getY());
    }

    public void setRotation(Rotation2d angle) {
        setPose(new Pose2d(getPose().getX(), getPose().getY(), angle));
    }

    public void zeroRotation() {
        setRotation(new Rotation2d());
    }

    public void setVelocity(ChassisSpeeds velocity, boolean isFieldOriented, boolean isOpenLoop) {
        driveSignal = new SwerveDriveSignal(velocity, isFieldOriented, isOpenLoop);
    }

    public void setVelocity(ChassisSpeeds velocity, boolean isFieldOriented) {
        setVelocity(velocity, isFieldOriented, true);
    }

    public void setVelocity(ChassisSpeeds velocity) {
        setVelocity(velocity, false);
    }

    public void stop() {
        driveSignal = new SwerveDriveSignal();
    }

    public void lock() {
        driveSignal = new SwerveDriveSignal(true);
    }

    public void update() {
        updateOdometry();

        if (isCharacterizing) return;

        updateModules(driveSignal);
    }

    private void updateOdometry() {
        SwerveModuleState[] moduleStates = getModuleStates();
        SwerveModulePosition[] modulePositions = getModulePositions();

        velocity = Constants.SwerveConstants.swerveKinematics.toChassisSpeeds(moduleStates);
        //SmartDashboard.putNumber("Robot X Velocity", velocity.vxMetersPerSecond);
        //SmartDashboard.putNumber("Robot Y Velocity", velocity.vyMetersPerSecond);

        velocityEstimator.add(velocity);

        pose = swervePoseEstimator.update(getGyroRotation(), modulePositions);
        m_field.setRobotPose(pose); //4143
    }

    private void updateModules(SwerveDriveSignal driveSignal) {
        ChassisSpeeds chassisVelocity;

        if (driveSignal.isFieldOriented()) {
            chassisVelocity = ChassisSpeeds.fromFieldRelativeSpeeds(
                    driveSignal.vxMetersPerSecond,
                    driveSignal.vyMetersPerSecond,
                    driveSignal.omegaRadiansPerSecond,
                    getRotation());
        } else {
            chassisVelocity = (ChassisSpeeds) driveSignal;
        }

        SwerveModuleState[] moduleStates =
                Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(chassisVelocity);

        if (driveSignal.isLocked()) {
            // get X for stopping
            moduleStates = new SwerveModuleState[] {
                new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
            };

            // Set the angle of each module only
            for (int i = 0; i < moduleStates.length; i++) {
                modules[i].setDesiredAngleOnly(moduleStates[i].angle, true);
            }
        } else {
            setModuleStates(moduleStates, isDriveSignalStopped(driveSignal) ? true : driveSignal.isOpenLoop());
        }
    }

    private void setModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.maxSpeed);

        for (SwerveModule module : modules) {
            module.setDesiredState(desiredStates[module.moduleNumber], isOpenLoop, isSecondOrder.getBoolean());
        }
    }

    private boolean isDriveSignalStopped(SwerveDriveSignal driveSignal) {
        return driveSignal.vxMetersPerSecond == 0
                && driveSignal.vyMetersPerSecond == 0
                && driveSignal.omegaRadiansPerSecond == 0;
    }

    @Override
    public void periodic() {
        update();
        // SmartDashboard.putNumber("X rate", gyro.getRotationRates3d().getX());
        // SmartDashboard.putNumber("Y rate", gyro.getRotationRates3d().getY());
        // SmartDashboard.putNumber("Z rate", gyro.getRotationRates3d().getZ());
        double pitch = getTiltAmount();
        SmartDashboard.putNumber("Pitch", pitch);
        Logger.log("/SwerveDriveSubsystem/Pose", pose);
        Logger.log("/SwerveDriveSubsystem/Velocity", velocity);
        

        Logger.log("/SwerveDriveSubsystem/Desired Velocity", (ChassisSpeeds) driveSignal);

        Logger.log("/SwerveDriveSubsystem/Velocity Magnitude", getVelocityMagnitude());

        Logger.log("/SwerveDriveSubsystem/Wheel Angles", new double[] {
            modules[0].getPosition().angle.getDegrees(),
            modules[1].getPosition().angle.getDegrees(),
            modules[2].getPosition().angle.getDegrees(),
            modules[3].getPosition().angle.getDegrees()
        });

        Logger.log("/SwerveDriveSubsystem/CANCoder Angles", new double[] {
            modules[0].getCanCoder().getDegrees(),
            modules[1].getCanCoder().getDegrees(),
            modules[2].getCanCoder().getDegrees(),
            modules[3].getCanCoder().getDegrees()
        });

        Logger.log("/SwerveDriveSubsystem/Drive Temperatures", getDriveTemperatures());
        Logger.log("/SwerveDriveSubsystem/Angle Temperatures", getAngleTemperatures());
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule module : modules) {
            states[module.moduleNumber] = module.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule module : modules) {
            positions[module.moduleNumber] = module.getPosition();
        }
        return positions;
    }

    public double[] getDriveTemperatures() {
        return new double[] {
            modules[0].getDriveTemperature(),
            modules[1].getDriveTemperature(),
            modules[2].getDriveTemperature(),
            modules[3].getDriveTemperature()
        };
    }

    public double[] getAngleTemperatures() {
        return new double[] {
            modules[0].getAngleTemperature(),
            modules[1].getAngleTemperature(),
            modules[2].getAngleTemperature(),
            modules[3].getAngleTemperature()
        };
    }
}
