package frc.robot.subsystems;

// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm extends SubsystemBase{
    // private CANSparkMax elevatorMotor;
    // private SparkMaxPIDController m_pidController;
    // private RelativeEncoder m_encoder;
    // public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
    private CANSparkMax clawMotor;
    private CANSparkMax elevatorMotor;
    private CANSparkMax rotatorMotor; 
    private DoubleSolenoid m_doubleSolenoid;
    private TalonFX toproller;
    private TalonFX bottomroller;
    private VictorSPX spindexter; 
    private SparkMaxPIDController m_pidController;
    private RelativeEncoder m_encoder;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
    private double distance;

    public Arm(){ 
        elevatorMotor = new CANSparkMax(10, MotorType.kBrushless);
        clawMotor = new CANSparkMax(3, MotorType.kBrushed);
        rotatorMotor = new CANSparkMax(5, MotorType.kBrushless);
        spindexter = new VictorSPX(10);
        toproller = new TalonFX(25);
        bottomroller = new TalonFX(26);

        m_doubleSolenoid =
            new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 3, 2);
            m_pidController = elevatorMotor.getPIDController();

    // Encoder object created to display position values
    m_encoder = elevatorMotor.getEncoder();

    // PID coefficients
    kP = 0.2; 
    kI = 1e-4;
    kD = 1; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 0.25; 
    kMinOutput = -0.25;
    distance = 0;

    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);
       

    }

    public void pickupRetract() {m_doubleSolenoid.set(DoubleSolenoid.Value.kForward);}

    public void pickupExtend() {m_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);}

    public void pickupOff() {m_doubleSolenoid.set(DoubleSolenoid.Value.kOff);}

    public void rollersin() {toproller.set(ControlMode.PercentOutput,-.40); bottomroller.set(ControlMode.PercentOutput,-.40);}
    public void rollersout() {toproller.set(ControlMode.PercentOutput,0.40); bottomroller.set(ControlMode.PercentOutput,0.40);}
    public void rollerstop() {toproller.set(ControlMode.PercentOutput,0); bottomroller.set(ControlMode.PercentOutput,0);}
    public void spindexterCW() {spindexter.set(ControlMode.PercentOutput,-.30);}
    public void spindexterCCW() {spindexter.set(ControlMode.PercentOutput,.30);}
    public void spindexterStop() {spindexter.set(ControlMode.PercentOutput,0);}

    
    public void setClawSpeed(double clawSpeed){
        clawMotor.set(clawSpeed);
    }

    public void setRotateSpeed(double rotateSpeed){
        rotatorMotor.set(rotateSpeed);
    }

    public void elevatorMove(double elevateSpeed){
       distance +=elevateSpeed;
       if(distance>0){
        distance=0;
       }
       if(distance<-88){
        distance=-88;
       }
    }


    @Override
    public void periodic(){
        SmartDashboard.putNumber("Arm distance", distance);
        m_pidController.setReference(distance, CANSparkMax.ControlType.kPosition);
    }
}
