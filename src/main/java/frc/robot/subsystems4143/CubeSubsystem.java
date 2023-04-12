package frc.robot.subsystems4143;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.container4143.CustomXboxController;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.controller.Axis;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class CubeSubsystem extends SubsystemBase {

    private CANSparkMax rackMotor;
    private VictorSPX rollerMotor;
    private VictorSPX beltMotor; 
    private RelativeEncoder m_cubeEncoder;
    public double cubekP, cubekI, cubekD ;
    private static final TrapezoidProfile.Constraints cubeConstraints = new TrapezoidProfile.Constraints(100, 100);
    private ProfiledPIDController rackMController;
    private double distance;
    private double count;
    DigitalInput input;
    double manualBeltPower;
    double autoBeltPower;


    public CubeSubsystem () {
        beltMotor = new VictorSPX(12);
        rollerMotor = new VictorSPX(11);
        rackMotor = new CANSparkMax( 4, MotorType.kBrushless);
        rackMotor.setSmartCurrentLimit(30);
        rackMotor.setInverted(false);
        rackMotor.setIdleMode(IdleMode.kBrake);
        m_cubeEncoder = rackMotor.getEncoder();
        cubekP = .4; 
        cubekI = 0.0;
        cubekD = .00;
        rackMController = new ProfiledPIDController(cubekP, cubekI, cubekD, cubeConstraints);
        m_cubeEncoder.setPositionConversionFactor(1);
        distance = 0;
        input = new DigitalInput(9);
        manualBeltPower = 0;
        autoBeltPower = 0;
        count = 0;
        beltMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void rollersSet(double speed) { 
        rollerMotor.set(ControlMode.PercentOutput, speed);
    }

    public void rackOut() {
        distance = -7;
        autoBeltPower = 1;
    }

    public void rackIn() {
        distance = 0;
    }

    public CommandBase pickupcancel() { return runOnce(() -> {rackIn();
        rollersSet(0);});}

    public CommandBase rollercancel() { return runOnce(() -> {rollersSet(0);});}

    public CommandBase rollerReverse() {
        return runEnd(() -> {rollersSet(0.5);}, 
        () -> {rollersSet(0);});
    }
    
    public CommandBase beltForward(Axis rightStick) {
        return new FunctionalCommand(() -> {}, () -> {manualBeltPower = ( -1* Math.abs(rightStick.get()));},
        interrupted -> {manualBeltPower = ( 0.0);}, () -> false);
    }

    public CommandBase beltReverse(Axis rightStick) {
        return new FunctionalCommand(() -> {}, () -> {manualBeltPower = (1* Math.abs(rightStick.get()));},
        interrupted -> {manualBeltPower = ( 0.0);}, () -> false);
    }

    public void beltStop() {
        beltMotor.set(ControlMode.PercentOutput, 0.0);
    }

    @Override
    public void periodic() {
        rackMController.setGoal(distance);
        //rackMotor.set(rackMController.calculate(m_cubeEncoder.getPosition()));
        double power = rackMController.calculate(m_cubeEncoder.getPosition());
        double error = distance - m_cubeEncoder.getPosition();

        if(error > .25 && distance > -1) {
            rackMotor.set(0.3);
        } else if(error < -.25 && distance < -1) {
            rackMotor.set(-0.6);
        } else {
            //rackMotor.set(power);
            rackMotor.set(0);
            
            
        }
        if(count > 0) 
            count--;
        
        if(input.get()) count = 3;
        if(count == 1) autoBeltPower = 0;
        if(manualBeltPower > 0) {
            autoBeltPower = 0;
        }
        if(autoBeltPower > 0)
            beltMotor.set(ControlMode.PercentOutput, autoBeltPower);
        else
            beltMotor.set(ControlMode.PercentOutput, manualBeltPower);

        
        SmartDashboard.putNumber("Rack Distance Actual", m_cubeEncoder.getPosition());
        SmartDashboard.putNumber("Rack Distance Commanded", distance);
        SmartDashboard.putNumber("Rack Power", rackMotor.get());
        SmartDashboard.putBoolean("cube sensor", input.get());

    }

}

