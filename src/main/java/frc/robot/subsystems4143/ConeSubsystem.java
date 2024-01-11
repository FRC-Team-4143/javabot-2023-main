package frc.robot.subsystems4143;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ConeSubsystem extends SubsystemBase{
    private CANSparkMax rollerMotor;
    private CANSparkMax extenderMotor;
    private double angle;
    public double conekP, conekI, conekD ;
    private RelativeEncoder extenderMotorEncoder;
    private ProfiledPIDController extenderMotorController;
    private static final TrapezoidProfile.Constraints coneConstraints = new TrapezoidProfile.Constraints(1000, 1000); //was 720 in qual 6

    //private static final TrapezoidProfile.Constraints cubeConstraints = new TrapezoidProfile.Constraints(100, 100);
   
    
    
    


    public ConeSubsystem () {
        extenderMotor = new CANSparkMax(7, MotorType.kBrushless);
        rollerMotor = new CANSparkMax(6, MotorType.kBrushless); // find out the motor id later
        rollerMotor.setInverted(false);
        extenderMotorEncoder = extenderMotor.getEncoder();
        extenderMotorEncoder.setPositionConversionFactor(360/(9*9*24/50));
        extenderMotor.setSmartCurrentLimit(10);
        rollerMotor.setSmartCurrentLimit(10);
        rollerMotor.setIdleMode(IdleMode.kBrake);
        extenderMotor.setIdleMode(IdleMode.kCoast);
        //rackMotor.setIdleMode(IdleMode.kBrake);
        conekP = 0.04;
        conekI = 0;
        conekD = 0.001;
        extenderMotorController = new ProfiledPIDController(conekP, conekI, conekD, coneConstraints);
        //beltMotor.setNeutralMode(NeutralMode.Brake);
        angle = 0;
        extenderMotor.enableVoltageCompensation(9);
        
    }

    public void setPickupMotorSpeed(double speed) { 
        rollerMotor.set(speed);
    }

    public double readExtenderEncoder(){
        return extenderMotorEncoder.getPosition();
    }

    public void setAngle(double angle){
        this.angle = angle;
        if(angle == 0) {
            rollerMotor.setIdleMode(IdleMode.kCoast);
            rollerMotor.set(1); //previously at .3
        } else {
            rollerMotor.setIdleMode(IdleMode.kBrake);
        }
    }

    public Command storePickup() { 
    return new FunctionalCommand(() -> {}, 
    () -> {
            angle = 0;
            setPickupMotorSpeed(0);
    }, interrupted -> {}, ()-> {
            return true;   
    });
    }

    // public Command intermediatePickup () { 
    // return new FunctionalCommand(() -> {}, 
    // () -> {
    //         angle = -106;
    // }, interrupted -> {}, ()-> {
    //     return true;
    // });}

    // public Command extendPickup() {
    // return new FunctionalCommand(() -> {}, 
    // () -> {
    //         angle = -168;
    // }, interrupted -> {}, ()-> {
    //         return false;
    // });
    // }
    
    public Command rollerReverse() {
        return runEnd(() -> {setPickupMotorSpeed(0); angle = -168;}, 
        () -> {setPickupMotorSpeed(0); angle = 0;});
    }

    public Command set0cone() {
        return runOnce(() -> {
            extenderMotorEncoder.setPosition(0);
            angle = 0;
            

        }).ignoringDisable(true);
    }

    public Command home() {
        return runEnd(() -> {extenderMotorEncoder.setPosition(0); extenderMotor.set(0.3); angle = 0;}, 
        () -> {extenderMotorEncoder.setPosition(0); extenderMotor.set(0); angle = 0;});
    }  

    @Override
    public void periodic() {
        extenderMotorController.setGoal(angle);
        extenderMotor.set(extenderMotorController.calculate(readExtenderEncoder()));
        if(readExtenderEncoder() > -1 && angle == 0) {
            extenderMotor.set(0);
            rollerMotor.set(0);
        }
        
        SmartDashboard.putNumber("Cone Angle", readExtenderEncoder());
        
        

    }
}