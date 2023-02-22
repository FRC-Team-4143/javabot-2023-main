package frc.robot.subsystems4143;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class SkiSubsystem extends SubsystemBase {
    // private SparkMaxPIDController m_pidController;
    // public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
   
    private TalonSRX skiMotor;
   

    public SkiSubsystem(){ 
        skiMotor = new TalonSRX(32);
        skiMotor.setNeutralMode(NeutralMode.Brake);
        skiMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    }

    
    public void setClawSpeed(double clawSpeed){
        skiMotor.set(ControlMode.PercentOutput, clawSpeed);
    }
   
    public CommandBase setSkiUp() {
            return runEnd(() -> {skiMotor.set(ControlMode.PercentOutput,-.30);}, 
            () -> skiMotor.set(ControlMode.PercentOutput, 0.0));
    }
    
    public CommandBase setSkiDown() {
        return runEnd(() -> {skiMotor.set(ControlMode.PercentOutput,0.30);}, 
        () -> skiMotor.set(ControlMode.PercentOutput, 0.0));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("skiangle", skiMotor.getSelectedSensorPosition());
    }
}
