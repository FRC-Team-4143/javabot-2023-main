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
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
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
    // private static final TrapezoidProfile.Constraints elevatorConstraints = new TrapezoidProfile.Constraints(28, 28);
    private static final TrapezoidProfile.Constraints elevatorConstraints = new TrapezoidProfile.Constraints(84, 84);
    private static final TrapezoidProfile.Constraints rotatorConstraints = new TrapezoidProfile.Constraints(360, 360);
    private Mechanism2d mechanism = new Mechanism2d(4, 4);
    private MechanismRoot2d root = mechanism.getRoot("Arm", 2, 2);
    private MechanismLigament2d arm1;
    private MechanismLigament2d arm2;
    public static final Rotation2d arm1StartingAngle = Rotation2d.fromDegrees(45);
    public static final Rotation2d arm2StartingAngle = Rotation2d.fromDegrees(-135);
    private double count;
    private boolean clamped;
    private ArmFeedforward m_rotatorFeedforward;
    private double armHomeHeight = -0.001;
    private double armHomeAngle = -14;

    public Arm(){ 
        clawMotor = new TalonSRX(3);
        clawMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        clawMotor.config_kP(0, 1, 10);
        clawMotor.config_kF(0, 0.005, 10);
        clawMotor.config_kI(0, 0.01, 10);
        clawMotor.config_kD(0, 0.001, 10);
        elevatorMotor = new CANSparkMax(11, MotorType.kBrushless);
        elevatorMotor2 = new CANSparkMax(10, MotorType.kBrushless);
        rotatorMotor = new CANSparkMax(5, MotorType.kBrushless);
        elevatorMotor.setSmartCurrentLimit(35);
        elevatorMotor2.setSmartCurrentLimit(35);
        rotatorMotor.setSmartCurrentLimit(35);
        m_rotatorEncoder = new WPI_CANCoder(1);
        m_rotatorEncoder.configFactoryDefault();
        elevatorMotor2.follow(elevatorMotor, true);
        count = 0;
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

    m_rotatorFeedforward = new ArmFeedforward(.159, .033, .033);

    arm1 = root.append(
                new MechanismLigament2d("Arm 1", 2.0, arm1StartingAngle.getDegrees()));
    arm2 = arm1.append(
                new MechanismLigament2d("Arm 2", 0.5, arm2StartingAngle.getDegrees()));

    arm1.setLineWeight(5);
    arm2.setLineWeight(5);

    // PID coefficients
    elevatorkP = 1.15*2; //18; 
    elevatorkI = 0.01;
    elevatorkD = .001;
    rotatorkP = 0.07; 
    rotatorkI = 0.01;
    rotatorkD = 0.001;
    distance = 0;
    angle = 0;

    // set PID coefficients
    elevatorMotorController = new ProfiledPIDController(elevatorkP, elevatorkI, elevatorkD, elevatorConstraints);
    rotatorMotorController = new ProfiledPIDController(rotatorkP, rotatorkI, rotatorkD, rotatorConstraints);
    }

    
    public void setClawSpeed(double clawSpeed){
        clawMotor.set(ControlMode.PercentOutput, clawSpeed);
    }

    public CommandBase setClawOpen(RobotContainer4143 container){
        return new FunctionalCommand(
            () -> {clamped = false; clawMotor.set(ControlMode.Current, 5);
                if(container.currentMode == gamePiece.Cube && distance <= -0.4) angle-=1;
            },
            () -> { clawMotor.set(ControlMode.Current, 5); }, 
            interrupted -> {clawMotor.set(ControlMode.Current, 0); },
            () -> (clawMotor.getSelectedSensorPosition() < 1000));
    }

    public CommandBase manualClawOpen(RobotContainer4143 container) {
        return runEnd(() -> {clawMotor.set(ControlMode.Current, 4); clamped = false;}, 
            () -> clawMotor.set(ControlMode.PercentOutput, 0.0));
    }

    public CommandBase setClawClosed(RobotContainer4143 container){
        return new FunctionalCommand(() -> {
                //PickupSubsystem pickup = container.getPickup();
                boolean driverLB = container.driver.getLeftBumper().getAsBoolean();
                boolean operatorLB = container.operator.getLeftBumper().getAsBoolean();
                if(angle > -20 && distance > -0.2 && !driverLB && !operatorLB) {
                    count = 2;
                    if(container.currentMode == gamePiece.Cone) {
                        //angle = -14;
                    } else {
                        //angle = -10;
                        //distance = -.04;
                    }
                    //pickup.solenoidStart();
                } else {
                    count = 0;
                }

                clamped = true;
            }, 
             () -> {
                if(count == 2 && readRotateEncoder() < angle + 2) {
                    //container.getPickup().solenoidStart();
                    count = 1;
                } 
                else if(count == 1) {
                     count = 0;
                }
                else if(count == 0){

                    count = -1;
                    if (container.currentMode == gamePiece.Cone) {
                        clawMotor.set(ControlMode.Current, -7);
                    }
                    else {
                        clawMotor.set(ControlMode.Current, -4);
                    }
                }   
            },
          interrupted -> {}, () -> (count == -1));
    }
    public boolean returnClamped() {
        return clamped;
    }

    public CommandBase clawToggle(RobotContainer4143 container) {
        return new ConditionalCommand(setClawOpen(container), setClawClosed(container), this::returnClamped);
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
           if(angle<-144){
            angle=-144;
           }
    }



    public CommandBase setHighPosition() {
        return new FunctionalCommand(() -> {}, 
        () -> {
            // if(m_elevatorEncoder.getPosition() > -0.44 ){
            //     distance = -0.727;
            //     angle = armHomeAngle;
            // } else {
                distance = -0.727;
                angle = -113;
            //}
        }, interrupted -> {}, ()-> {
            if(readRotateEncoder() < -105 && angle == -113 && distance == -0.727){
                return true;
            }else{
                return false;
            }
        });
    }


    public CommandBase setMidPosition() {
        return new FunctionalCommand(() -> {}, 
        () -> {
            //if(m_elevatorEncoder.getPosition() > -0.4 ){
                //distance = -0.5;
                //angle = armHomeAngle;
            //} else {
                distance = -0.5;
                angle = -101;
            //}
        }, interrupted -> {}, ()-> {
            if(angle == -101 && distance == -0.5){
                return true;
            }else{
                return false;
            }
        });
    }

    public CommandBase setHybridPosition(PickupSubsystem pickup) {
        return new FunctionalCommand(() -> {}, 
        () -> {
            if(m_elevatorEncoder.getPosition() < -0.35 || readRotateEncoder() < -55. || pickup.m_doubleSolenoid0.get().equals(Value.kForward)){
                angle = -114.;
            } else {
                distance = -.4;
            }
            if (readRotateEncoder() < -110) {
                distance = -0.01;
            }
        }, interrupted -> {}, ()-> {
            if(angle == -114 && distance == -0.01){
                return true;
            }else{
                return false;
            }
        });
    }

    public CommandBase setHybridPosition() {
        return new FunctionalCommand(() -> {}, 
        () -> {
            if(m_elevatorEncoder.getPosition() < -0.35 || readRotateEncoder() < -55.){
                angle = -114.;
            } else {
                distance = -.4;
            }
            if (readRotateEncoder() < -110) {
                distance = -0.01;
            }
        }, interrupted -> {}, ()-> {
            if(angle == -114 && distance == -0.01){
                return true;
            }else{
                return false;
            }
        });
    }

    public CommandBase setHomePosition() {
        return new FunctionalCommand(() -> {}, 
        () -> {
            //if( readRotateEncoder() > -45.) {
                distance = armHomeHeight; angle = armHomeAngle;
            //} else if(m_elevatorEncoder.getPosition() < -.35) {
            //    angle = armHomeAngle;
            //} else {
            //    distance = -.4;
            //}
        }, interrupted -> {}, ()-> {
            if(angle == armHomeAngle && distance == armHomeHeight){
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
        if(distance>= armHomeHeight){
            distance= armHomeHeight;
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
        m_rotatorFeedforward.calculate(Math.toRadians(angle), 180);
        arm1.setLength(2.0 - m_elevatorEncoder.getPosition());
        arm2.setAngle(readRotateEncoder());

        elevatorMotorController.setGoal(distance);
        elevatorMotor.set(elevatorMotorController.calculate(m_elevatorEncoder.getPosition()));

        rotatorMotorController.setGoal(angle);
        rotatorMotor.set(rotatorMotorController.calculate(readRotateEncoder()));
        //m_pidController.setReference(distance, CANSparkMax.ControlType.kPosition);

        SmartDashboard.putNumber("Arm distance", m_elevatorEncoder.getPosition());
        SmartDashboard.putNumber("Arm angle", readRotateEncoder());

        //SmartDashboard.putData("Arm Mechanism", mechanism);
        // SmartDashboard.putNumber("Distance Setpoint", distance);
        // SmartDashboard.putNumber("Angle Setpoint", angle);
        // SmartDashboard.putNumber("Rotator Current", rotatorMotor.getOutputCurrent());
        // SmartDashboard.putNumber("Rotator Speed", m_rotatorEncoder.getVelocity());
        // SmartDashboard.putNumber("Elevator Current", elevatorMotor.getOutputCurrent());
        // SmartDashboard.putNumber("Elevator Voltage", elevatorMotor.getAppliedOutput());
        // SmartDashboard.putNumber("Elevator Speed", m_elevatorEncoder.getVelocity());
        SmartDashboard.putNumber("Claw Angle", clawMotor.getSelectedSensorPosition());
        // SmartDashboard.putBoolean("Clamped", clamped);
        // SmartDashboard.putNumber("Claw Current", clawMotor.getStatorCurrent());
        SmartDashboard.putNumber("Arm extension", (Math.cos(Math.toRadians(41)) * -m_elevatorEncoder.getPosition()) +
                        (Math.cos(Math.toRadians(readRotateEncoder() + 114)) * 0.762) 
                        - 0.1016)
                        ;

    }
}
