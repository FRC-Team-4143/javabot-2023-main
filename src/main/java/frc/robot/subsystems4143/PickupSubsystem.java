package frc.robot.subsystems4143;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class PickupSubsystem extends SubsystemBase {
    private DoubleSolenoid m_doubleSolenoid0;
    private DoubleSolenoid m_doubleSolenoid1;
    private DoubleSolenoid m_doubleSolenoid2;
    private CANSparkMax toproller;
    private CANSparkMax bottomroller;
    private VictorSPX spindexter; 


    public PickupSubsystem () {
        spindexter = new VictorSPX(10);
        toproller = new CANSparkMax(25, MotorType.kBrushless);
        bottomroller = new CANSparkMax(26, MotorType.kBrushless);

        m_doubleSolenoid0 = new DoubleSolenoid(PneumaticsModuleType.REVPH,0, 1); //short cylinder
        m_doubleSolenoid1 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 2, 3); //dump cylinder
        m_doubleSolenoid2 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 4, 5);//long cylinder
        
    }


    public void solenoidStart() {m_doubleSolenoid0.set(Value.kForward);
                                m_doubleSolenoid1.set(Value.kReverse);   //Reverse is non - dump    , Forward is dump
                                m_doubleSolenoid2.set(Value.kForward);} 
    public void solenoidRetract() {m_doubleSolenoid0.set(Value.kForward);
                                   m_doubleSolenoid1.set(Value.kReverse);
                                   m_doubleSolenoid2.set(Value.kReverse);}
    public void solenoidExtend() {m_doubleSolenoid0.set(Value.kReverse);
                                  m_doubleSolenoid1.set(Value.kReverse);
                                  m_doubleSolenoid2.set(Value.kReverse);}
    public void rollersSet(double speed) {toproller.set(speed); 
        bottomroller.set(speed);}
    //public void pickupOff() {m_doubleSolenoid.set(DoubleSolenoid.Value.kOff);}

    // public CommandBase pickupExtend() {
    //     return new FunctionalCommand (
    //         () -> {m_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);}, () -> {},
    //         interrupted ->  {m_doubleSolenoid.set(DoubleSolenoid.Value.kOff);}, () -> false);
    // }

    // public CommandBase pickupRetract() {
    //     return new FunctionalCommand(() -> {m_doubleSolenoid.set(DoubleSolenoid.Value.kForward);}, () -> {},interrupted ->  {}, () -> false);
    // }

    public CommandBase pickupcancel() { return runOnce(() -> {});}

    public CommandBase rollIn() {
        return new FunctionalCommand(() -> {toproller.set(-.40); 
                        bottomroller.set(-.40);}, () -> {},
                      interrupted -> {toproller.set(0); 
                        bottomroller.set(0);}, () -> false
        );
    }

    public CommandBase rollOut() {
        return new FunctionalCommand(() -> {toproller.set(.40); 
                        bottomroller.set(.40);}, () -> {},
                      interrupted -> {toproller.set(0); 
                        bottomroller.set(0);}, () -> false
        );
    }

    // public CommandBase pickupOut() {
    //     return new FunctionalCommand(() -> {toproller.set(ControlMode.PercentOutput,-.40); 
    //         bottomroller.set(ControlMode.PercentOutput,-.40);
    //         m_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);}, () -> {},
    //         interrupted -> {toproller.set(ControlMode.PercentOutput,0);
    //         bottomroller.set(ControlMode.PercentOutput,0);
    //         m_doubleSolenoid.set(DoubleSolenoid.Value.kForward);}, null, null)
    // }

    public CommandBase spindexterCW() {
        return runEnd(() -> {spindexter.set(ControlMode.PercentOutput,-.50);}, 
        () -> spindexter.set(ControlMode.PercentOutput, 0.0));}
    public CommandBase spindexterCCW() {
        return runEnd(() -> {spindexter.set(ControlMode.PercentOutput,.50);},
        () -> spindexter.set(ControlMode.PercentOutput, 0.0));} 
    public void spindexterStop() {
        spindexter.set(ControlMode.PercentOutput, 0.0);
    }

}

