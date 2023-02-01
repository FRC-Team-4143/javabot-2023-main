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

    public Arm(){ 
        elevatorMotor = new CANSparkMax(10, MotorType.kBrushless);
        clawMotor = new CANSparkMax(3, MotorType.kBrushed);
        rotatorMotor = new CANSparkMax(5, MotorType.kBrushless);
        spindexter = new VictorSPX(10);
        toproller = new TalonFX(25);
        bottomroller = new TalonFX(26);

        m_doubleSolenoid =
            new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 3, 2);
        
        // m_pidController = elevatorMotor.getPIDController();
        // m_encoder = elevatorMotor.getEncoder();

    // // PID coefficients
    // kP = 5e-5; 
    // kI = 1e-6;
    // kD = 0; 
    // kIz = 0; 
    // kFF = 0.000156; 
    // kMaxOutput = 1; 
    // kMinOutput = -1;
    // maxRPM = 5700;
    // // Smart Motion Coefficients
    // maxVel = 2000; // rpm
    // maxAcc = 1500;
    // // set PID coefficients
    // m_pidController.setP(kP);
    // m_pidController.setI(kI);
    // m_pidController.setD(kD);
    // m_pidController.setIZone(kIz);
    // m_pidController.setFF(kFF);
    // m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    }

    public void pickupRetract() {m_doubleSolenoid.set(DoubleSolenoid.Value.kForward);}

    public void pickupExtend() {m_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);}

    public void pickupOff() {m_doubleSolenoid.set(DoubleSolenoid.Value.kOff);}

    public void rollersin() {toproller.set(ControlMode.PercentOutput,-.75); bottomroller.set(ControlMode.PercentOutput,-.50);}
    public void rollersout() {toproller.set(ControlMode.PercentOutput,1.0); bottomroller.set(ControlMode.PercentOutput,1.0);}
    public void rollerstop() {toproller.set(ControlMode.PercentOutput,0); bottomroller.set(ControlMode.PercentOutput,0);}
    public void spindexterCW() {spindexter.set(ControlMode.PercentOutput,-.50);}
    public void spindexterCCW() {spindexter.set(ControlMode.PercentOutput,.50);}
    public void spindexterStop() {spindexter.set(ControlMode.PercentOutput,0);}

    
    public void setClawSpeed(double clawSpeed){
        clawMotor.set(clawSpeed);
    }

    public void setRotateSpeed(double rotateSpeed){
        rotatorMotor.set(rotateSpeed);
    }

    public void setElevatorSpeed(double elevateSpeed){
        elevatorMotor.set(elevateSpeed);
    }
}
