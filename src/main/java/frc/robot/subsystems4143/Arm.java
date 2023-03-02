package frc.robot.subsystems4143;

import java.sql.Timestamp;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer4143;
import frc.robot.Constants.gamePiece;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.TimestampedInteger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.math.geometry.Rotation2d;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;

public class Arm extends SubsystemBase {
    // private SparkMaxPIDController m_pidController;
    // public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
    private ProfiledPIDController elevatorMotorController;
    private ProfiledPIDController rotatorMotorController;
    private TalonSRX clawMotor;
    private CANSparkMax elevatorMotor;
    private CANSparkMax rotatorMotor; 
    private CANSparkMax elevatorMotor2;
    private WPI_CANCoder m_rotatorEncoder; 
    //private SparkMaxPIDController m_pidController;
    private RelativeEncoder m_elevatorEncoder;
    private RelativeEncoder m_rotatorEncoderMotor;
    public double elevatorkP, rotatorkP, elevatorkI, rotatorkI, elevatorkD, rotatorkD;
    private double distance;
    private double angle;
    // private static final TrapezoidProfile.Constraints elevatorConstraints = new TrapezoidProfile.Constraints(84, 84);
    private static final TrapezoidProfile.Constraints elevatorConstraints = new TrapezoidProfile.Constraints(28, 28);
    private static final TrapezoidProfile.Constraints rotatorConstraints = new TrapezoidProfile.Constraints(180, 180);
    private Mechanism2d mechanism = new Mechanism2d(4, 4);
    private MechanismRoot2d root = mechanism.getRoot("Arm", 2, 2);
    private MechanismLigament2d arm1;
    private MechanismLigament2d arm2;
    public static final Rotation2d arm1StartingAngle = Rotation2d.fromDegrees(45);
    public static final Rotation2d arm2StartingAngle = Rotation2d.fromDegrees(-135);
    private double count;
    private boolean clamped;

    public Arm(){ 
        clawMotor = new TalonSRX(3);
        clawMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        elevatorMotor = new CANSparkMax(11, MotorType.kBrushless);
        elevatorMotor2 = new CANSparkMax(10, MotorType.kBrushless);
        rotatorMotor = new CANSparkMax(5, MotorType.kBrushless);
        elevatorMotor.setSmartCurrentLimit(20);
        rotatorMotor.setSmartCurrentLimit(20);
        m_rotatorEncoder = new WPI_CANCoder(1);
        m_rotatorEncoder.configFactoryDefault();
    elevatorMotor2.follow(elevatorMotor, true);
    count =0;
        clamped = false;

    if (clawMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute) != ErrorCode.OK) {
        System.out.println("Claw encoder error");
    }
    clawMotor.configFeedbackNotContinuous(true, 0);
    clawMotor.setNeutralMode(NeutralMode.Brake);
        
    // Encoder object created to display position values
    m_elevatorEncoder = elevatorMotor.getEncoder();
    m_rotatorEncoderMotor = rotatorMotor.getEncoder();
    m_elevatorEncoder.setPositionConversionFactor(22*0.25/9*(25.4/1000)); //Conversion from revolutions to meters, 0.01129
    m_rotatorEncoderMotor.setPositionConversionFactor(360/100); //Conversion from revolutions to degrees, 3.6

    arm1 = root.append(
                new MechanismLigament2d("Arm 1", 2.0, arm1StartingAngle.getDegrees()));
    arm2 = arm1.append(
                new MechanismLigament2d("Arm 2", 0.5, arm2StartingAngle.getDegrees()));

    arm1.setLineWeight(5);
    arm2.setLineWeight(5);

    // PID coefficients
    elevatorkP = 1.15; //18; 
    elevatorkI = 0;
    elevatorkD = 0;
    rotatorkP = 0.1; 
    rotatorkI = 0;
    rotatorkD = 0;
    distance = 0;
    angle = 0;

    // set PID coefficients
    elevatorMotorController = new ProfiledPIDController(elevatorkP, elevatorkI, elevatorkD, elevatorConstraints);
    rotatorMotorController = new ProfiledPIDController(rotatorkP, rotatorkI, rotatorkD, rotatorConstraints);
    }

    
    public void setClawSpeed(double clawSpeed){
        clawMotor.set(ControlMode.PercentOutput, clawSpeed);
    }

    public CommandBase setClawOpen(){
        return new FunctionalCommand(
            () -> {clamped = false; clawMotor.set(ControlMode.PercentOutput, 1);},
            () -> { clawMotor.set(ControlMode.PercentOutput, 1); System.out.println("Moved Claw Out");}, 
            interrupted -> {clawMotor.set(ControlMode.PercentOutput, 0); System.out.println("Motor Off");},
            () -> (clawMotor.getSelectedSensorPosition() < 3200 && clawMotor.getSelectedSensorPosition() > 1400));
    }

    public CommandBase setClawClosed(RobotContainer4143 container){
        return new FunctionalCommand(() -> {count = Timer.getFPGATimestamp(); clamped = true;}, 
             () -> {
                clawMotor.set(ControlMode.Current, -3);
                if(count + 0.03 < Timer.getFPGATimestamp() && getDistance() < -0.29){

            } 
            if(count + 0.25 < Timer.getFPGATimestamp()){
                if (container.currentMode == gamePiece.Cone) {
                    // setClawSpeed(-0.75);
                    clawMotor.set(ControlMode.Current, -6);
                }
                else if (container.currentMode == gamePiece.Cube){
                    clawMotor.set(ControlMode.Current, -3);
                }
            } else {
                clawMotor.set(ControlMode.Current, -3);
            }},

          interrupted -> {clawMotor.set(ControlMode.Current, -3);}, () -> (count + 0.5 < Timer.getFPGATimestamp())
    );

    }
    public boolean returnClamped() {
        return clamped;
    }

    public CommandBase clawToggle(RobotContainer4143 container) {
        return new ConditionalCommand(setClawOpen(), setClawClosed(container), this::returnClamped);
    }


    public double readRotateEncoder() {
        //return m_rotatorEncoder.getAbsolutePosition() + 106 - 360;
        return m_rotatorEncoderMotor.getPosition();
    }

    public void setRotate(double rotateSpeed){ //Make this two functional
        angle += rotateSpeed;
        if(angle>0){
            angle=0;
           }
           if(angle<-180){
            angle=-180;
           }
    }



    public CommandBase setHighPosition() {
        return new FunctionalCommand(() -> {}, 
        () -> {
            if(m_elevatorEncoder.getPosition() > -0.44 ){
                distance = -0.7;
            } else {
                distance = -0.7;
                angle = -115;
            }
        }, interrupted -> {}, ()-> {
            if(readRotateEncoder() < -110 && angle == -115 && distance == -0.7){
                return true;
            }else{
                return false;
            }
        });
    }

    public CommandBase setMidPosition() {
        return new FunctionalCommand(() -> {}, 
        () -> {
            if(m_elevatorEncoder.getPosition() > -0.4 ){
                distance = -0.5;
            } else {
                distance = -0.5;
                angle = -101;
            }
        }, interrupted -> {}, ()-> {
            if(angle == -101 && distance == -0.5){
                return true;
            }else{
                return false;
            }
        });
    }

    public CommandBase setHybridPosition() {
        return new FunctionalCommand(() -> {}, 
        () -> {
            if(m_elevatorEncoder.getPosition() > -0.38 ){
                distance = -0.4;
            } else {
                distance = -0.4;
                angle = -95;
            }
        }, interrupted -> {}, ()-> {
            if(angle == -95 && distance == -0.4){
                return true;
            }else{
                return false;
            }
        });
    }

    public CommandBase setHomePosition() {
        return new FunctionalCommand(() -> {}, 
        () -> {
            if(readRotateEncoder() < -5.4 ){
                angle = -3.6;
            } else {
                distance = -0.1;
                angle = -3.6;
            }
        }, interrupted -> {}, ()-> {
            if(angle == -3.6 && distance == -0.1){
                return true;
            }else{
                return false;
            }
            
        });
    }

    public CommandBase set0Arm() {
        return runOnce(() -> {
            m_elevatorEncoder.setPosition(0);
            m_rotatorEncoderMotor.setPosition(0);
            distance=0;
            angle=0;

        }).ignoringDisable(true);
    }

    public void elevatorMove(double elevateSpeed){
       distance +=elevateSpeed;
       if(distance>0){
        distance=0;
       }
       if(distance<-0.75){
        distance=-0.75;
       }
    }
    public void elevatorPickup(){
        if(distance>-0.3){
            distance=-0.3;
        }
    }

    public double getDistance(){
        return distance;
    }
    public void setPosition(){
        distance = m_elevatorEncoder.getPosition();
        angle = readRotateEncoder();
        elevatorMotorController.setGoal(distance);
        rotatorMotorController.setGoal(angle);
    }

    @Override
    public void periodic(){
        arm1.setLength(2.0 - m_elevatorEncoder.getPosition());
        arm2.setAngle(readRotateEncoder());
        elevatorMotorController.setGoal(distance);
        elevatorMotor.set(elevatorMotorController.calculate(m_elevatorEncoder.getPosition()));
        rotatorMotorController.setGoal(angle);
        rotatorMotor.set(rotatorMotorController.calculate(readRotateEncoder()));
        //m_pidController.setReference(distance, CANSparkMax.ControlType.kPosition);

        SmartDashboard.putNumber("Arm distance", m_elevatorEncoder.getPosition());
        SmartDashboard.putNumber("Arm angle", readRotateEncoder());
        SmartDashboard.putData("Arm Mechanism", mechanism);
        SmartDashboard.putNumber("Distance Setpoint", distance);
        SmartDashboard.putNumber("Angle Setpoint", angle);
        SmartDashboard.putNumber("Elevator Current", elevatorMotor.getOutputCurrent());
        SmartDashboard.putNumber("Elevator Voltage", elevatorMotor.getAppliedOutput());
        SmartDashboard.putNumber("Claw Angle", clawMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("Arm extension", 1000 / 25.4 * (Math.cos(Math.toRadians(47)) * -distance +
                        Math.cos(Math.toRadians(angle + 135) * 0.4953) - 0.076));
    }
}
