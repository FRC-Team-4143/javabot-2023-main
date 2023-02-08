package frc.robot.subsystems;

// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.math.geometry.Rotation2d;


public class Arm extends SubsystemBase {
    // private CANSparkMax elevatorMotor;
    // private SparkMaxPIDController m_pidController;
    // public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
    private ProfiledPIDController elevatorMotorController;
    private ProfiledPIDController rotatorMotorController;
    private CANSparkMax clawMotor;
    private CANSparkMax elevatorMotor;
    private CANSparkMax rotatorMotor; 
    private DoubleSolenoid m_doubleSolenoid;
    private TalonFX toproller;
    private TalonFX bottomroller;
    private VictorSPX spindexter; 
    //private SparkMaxPIDController m_pidController;
    private RelativeEncoder m_elevatorEncoder;
    private RelativeEncoder m_rotatorEncoder;
    public double elevatorkP, rotatorkP, elevatorkI, rotatorkI, elevatorkD, rotatorkD, kMaxOutput, kMinOutput;
    private double distance;
    private double angle;
    private static final TrapezoidProfile.Constraints elevatorConstraints = new TrapezoidProfile.Constraints(84, 84);
    private static final TrapezoidProfile.Constraints rotatorConstraints = new TrapezoidProfile.Constraints(20, 20);
    private Mechanism2d mechanism = new Mechanism2d(4, 4);
    private MechanismRoot2d root = mechanism.getRoot("Arm", 2, 2);
    private MechanismLigament2d arm1;
    private MechanismLigament2d arm2;
    public static final Rotation2d arm1StartingAngle = Rotation2d.fromDegrees(45);
    public static final Rotation2d arm2StartingAngle = Rotation2d.fromDegrees(-135);


    public Arm(){ 
        elevatorMotor = new CANSparkMax(10, MotorType.kBrushless);
        clawMotor = new CANSparkMax(3, MotorType.kBrushed);
        rotatorMotor = new CANSparkMax(5, MotorType.kBrushless);
        spindexter = new VictorSPX(10);
        toproller = new TalonFX(25);
        bottomroller = new TalonFX(26);
        elevatorMotor.setSmartCurrentLimit(20);

        m_doubleSolenoid =
            new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 3, 2);
            //m_pidController = elevatorMotor.getPIDController();
            

    // Encoder object created to display position values
    m_elevatorEncoder = elevatorMotor.getEncoder();
    m_rotatorEncoder = rotatorMotor.getEncoder();

    arm1 = root.append(
                new MechanismLigament2d("Arm 1", 2.0, arm1StartingAngle.getDegrees()));
    arm2 = arm1.append(
                new MechanismLigament2d("Arm 2", 0.5, arm2StartingAngle.getDegrees()));

    arm1.setLineWeight(5);
    arm2.setLineWeight(5);

    // PID coefficients
    elevatorkP = 0.2; 
    elevatorkI = 0;
    elevatorkD = 0;
    rotatorkP = 0.12; 
    rotatorkI = 0;
    rotatorkD = 0;
    kMaxOutput = 0.25; 
    kMinOutput = -0.25;
    distance = 0;
    angle = 0;

    // set PID coefficients
    elevatorMotorController = new ProfiledPIDController(elevatorkP, elevatorkI, elevatorkD, elevatorConstraints);
    rotatorMotorController = new ProfiledPIDController(rotatorkP, rotatorkI, rotatorkD, rotatorConstraints);


    }

    public void pickupRetract() {m_doubleSolenoid.set(DoubleSolenoid.Value.kForward);}

    public void pickupExtend() {m_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);}

    public void pickupOff() {m_doubleSolenoid.set(DoubleSolenoid.Value.kOff);}

    public void rollersin() {toproller.set(ControlMode.PercentOutput,-.40); bottomroller.set(ControlMode.PercentOutput,-.40);}
    public void rollersout() {toproller.set(ControlMode.PercentOutput,0.40); bottomroller.set(ControlMode.PercentOutput,0.40);}
    public void rollerstop() {toproller.set(ControlMode.PercentOutput,0); bottomroller.set(ControlMode.PercentOutput,0);}
    public CommandBase spindexterCW() {
        return runEnd(() -> {spindexter.set(ControlMode.PercentOutput,-.30);}, 
        () -> spindexter.set(ControlMode.PercentOutput, 0.0));}
    public CommandBase spindexterCCW() {
        return runEnd(() -> {spindexter.set(ControlMode.PercentOutput,.30);},
        () -> spindexter.set(ControlMode.PercentOutput, 0.0));} 
    public void spindexterStop() {
        spindexter.set(ControlMode.PercentOutput, 0.0);
    }

    
    public void setClawSpeed(double clawSpeed){
        clawMotor.set(clawSpeed);
    }

    public void setRotateSpeed(double rotateSpeed){
        angle += rotateSpeed;
        if(angle>0){
            angle=0;
           }
           if(angle<-42){
            angle=-42;
           }
    }

    public CommandBase setHighPosition() {
        return new FunctionalCommand(() -> {}, 
        () -> {
            if(m_elevatorEncoder.getPosition() > -39 ){
                distance = -80;
            } else {
                distance = -80;
                angle = -42;
            }
        }, interrupted -> {}, ()-> {
            if(angle == -42 && distance == -80){
                return true;
            }else{
                return false;
            }
        });
    }

    public CommandBase setMidPosition() {
        return new FunctionalCommand(() -> {}, 
        () -> {
            if(m_elevatorEncoder.getPosition() > -39 ){
                distance = -50;
            } else {
                distance = -50;
                angle = -34;
            }
        }, interrupted -> {}, ()-> {
            if(angle == -34 && distance == -50){
                return true;
            }else{
                return false;
            }
        });
    }

    public CommandBase setHomePosition() {
        return new FunctionalCommand(() -> {}, 
        () -> {
            if(m_rotatorEncoder.getPosition() < -1.5 ){
                angle = -1;
            } else {
                distance = -1;
                angle = -1;
            }
        }, interrupted -> {}, ()-> {
            if(angle == -1 && distance == -1){
                return true;
            }else{
                return false;
            }
            
        });
    }

    public CommandBase set0Arm() {
        return runOnce(() -> {
            m_elevatorEncoder.setPosition(0);
            m_rotatorEncoder.setPosition(0);
        }).ignoringDisable(true);
    }

    public void elevatorMove(double elevateSpeed){
       distance +=elevateSpeed;
       if(distance>0){
        distance=0;
       }
       if(distance<-84){
        distance=-84;
       }
    }

    public void setPosition(){
        distance = m_elevatorEncoder.getPosition();
        angle = m_rotatorEncoder.getPosition();
        elevatorMotorController.setGoal(distance);
        rotatorMotorController.setGoal(angle);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Arm distance", m_elevatorEncoder.getPosition());
        SmartDashboard.putNumber("Arm angle", m_rotatorEncoder.getPosition());
        SmartDashboard.putData("Arm Mechanism", mechanism);
        SmartDashboard.putNumber("Distance Setpoint", distance);
        SmartDashboard.putNumber("Angle Setpoint", angle);
        SmartDashboard.putNumber("Elevator Current", elevatorMotor.getOutputCurrent());
        SmartDashboard.putNumber("Elevator Voltage", elevatorMotor.getAppliedOutput());

        elevatorMotorController.setGoal(distance);
        elevatorMotor.set(elevatorMotorController.calculate(m_elevatorEncoder.getPosition()));
        rotatorMotorController.setGoal(angle);
        rotatorMotor.set(rotatorMotorController.calculate(m_rotatorEncoder.getPosition()));
        //m_pidController.setReference(distance, CANSparkMax.ControlType.kPosition);
    }
}
