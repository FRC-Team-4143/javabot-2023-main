package frc.robot.subsystems4143;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer4143;
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

    public int state;
    public boolean lastSensor;
    private CANSparkMax rackMotor;
    private VictorSPX rollerMotor;
    private VictorSPX beltMotor; 
    private RelativeEncoder m_cubeEncoder;
    public double cubekP, cubekI, cubekD ;
    private static final TrapezoidProfile.Constraints cubeConstraints = new TrapezoidProfile.Constraints(100, 100);
    private ProfiledPIDController rackMController;
    private double distance;
    private double count;
    private int stuckCube;
    private int setCube;
    //private Counter counter; 
    DigitalInput input;
    double manualBeltPower;
    double autoBeltPower;
    RobotContainer4143 container;



    public CubeSubsystem (RobotContainer4143 container) {
        state = 0;
        lastSensor = false;
        
        beltMotor = new VictorSPX(12);
        rollerMotor = new VictorSPX(11);
        rackMotor = new CANSparkMax(4, MotorType.kBrushless);
        rackMotor.setSmartCurrentLimit(40);
        rackMotor.setInverted(false);
        rackMotor.setIdleMode(IdleMode.kBrake);
        m_cubeEncoder = rackMotor.getEncoder();
        cubekP = .4; 
        cubekI = 0.0;
        cubekD = .00;
        rackMController = new ProfiledPIDController(cubekP, cubekI, cubekD, cubeConstraints);
        rackMotor.enableVoltageCompensation(11);
        m_cubeEncoder.setPositionConversionFactor(1);
        distance = 0;
        input = new DigitalInput(9);
        //counter = new Counter(Counter.Mode.kTwoPulse);
        //counter.setUpSource(input);
        //counter.setDownSource(input);
        //counter.setUpSourceEdge(true, false);
        //counter.setDownSourceEdge(false, false);
        manualBeltPower = 0;
        autoBeltPower = 0;
        count = 0;
        stuckCube = 0;
        setCube = 0;
        beltMotor.setNeutralMode(NeutralMode.Brake);
        this.container = container;
    }

    public void rollersSet(double speed) { 
        rollerMotor.set(ControlMode.PercentOutput, speed);
    }

    public void rackOut() {
        distance = -5.5;
        autoBeltPower = 0.85; //was 1 in qual 17
        state = 0;
    }

    public void rackIn() {
        distance = 0;
    }

    public CommandBase pickupcancel() { return runOnce(() -> {rackIn();
        rollersSet(0);});}

    public CommandBase rollercancel() { return runOnce(() -> {rollersSet(0);});}

    public CommandBase rollerReverse() {
        return runEnd(() -> {rollersSet(0.7);}, 
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

    public CommandBase cubeDetected() {
        return new FunctionalCommand(() -> {stuckCube = 1;}, () -> {
            if(stuckCube % 15 == 0) distance = -1.5; 
            if(stuckCube % 30 == 0) distance = 0;
            stuckCube++; },
        interrupted -> {distance = 0; autoBeltPower = 0;}, () -> state >= 2);
    }

    public CommandBase shootCube(RobotContainer4143 container) {
        return new FunctionalCommand(() -> {setCube = 0;}, () -> {
            if(setCube == 0) { 
                beltMotor.set(ControlMode.PercentOutput, -0.5); 
                container.getArm().distance = -.25; 
            }
            if(setCube == 40 ) {
                beltMotor.set(ControlMode.PercentOutput, 1);
            }
            if(setCube == 70) {
                beltMotor.set(ControlMode.PercentOutput, 0);
            }
            setCube++;
             },
        interrupted -> {beltMotor.set(ControlMode.PercentOutput, 0); 
                container.getArm().clawMotor.set(ControlMode.Current, 0);}, () -> setCube >= 50);
    }

    public void beltStop() {
        beltMotor.set(ControlMode.PercentOutput, 0.0);
    }


    public CommandBase set0cube() {
        return runOnce(() -> {
            m_cubeEncoder.setPosition(0);
            distance=0;
            

        }).ignoringDisable(true);
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
            
        if(lastSensor == false && input.get() == true) 
            state++;
        
        if(lastSensor && input.get() == false)
            state++;
            
        lastSensor = input.get();
        
        if(count > 0) 
            count--;
        //SmartDashboard.putNumber("Sensor Counter", counter.get());
        if(input.get()) count = 2; //was 3 qual 17
        if(count == 1) { autoBeltPower = 0; container.getArm().setClawClosed(container).schedule(); }
        if(Math.abs(manualBeltPower) > 0) {
            autoBeltPower = 0;
        }
        //counter.reset();
        if(autoBeltPower > 0)
            beltMotor.set(ControlMode.PercentOutput, autoBeltPower);
        else
            beltMotor.set(ControlMode.PercentOutput, manualBeltPower);

        
        SmartDashboard.putNumber("Rack Distance Actual", m_cubeEncoder.getPosition());
        SmartDashboard.putNumber("Rack Distance Commanded", distance);
        SmartDashboard.putNumber("Rack Power", rackMotor.get());
        SmartDashboard.putBoolean("cube sensor", input.get());
        SmartDashboard.putNumber("Times Caught", state);
        }

    }
}

