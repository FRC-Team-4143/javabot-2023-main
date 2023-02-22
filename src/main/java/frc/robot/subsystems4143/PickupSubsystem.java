package frc.robot.subsystems4143;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class PickupSubsystem extends SubsystemBase {
    private Solenoid m_singleSolenoid0;
    private Solenoid m_singleSolenoid1;
    private CANSparkMax toproller;
    private CANSparkMax bottomroller;
    private VictorSPX spindexter; 


    public PickupSubsystem () {
        spindexter = new VictorSPX(10);
        toproller = new CANSparkMax(25, MotorType.kBrushless);
        bottomroller = new CANSparkMax(26, MotorType.kBrushless);

        m_singleSolenoid0 = new Solenoid(PneumaticsModuleType.REVPH, 0);
        m_singleSolenoid1 = new Solenoid(PneumaticsModuleType.REVPH, 1);
    }

    public void solenoidRetract() {m_singleSolenoid0.set(false);
                                   m_singleSolenoid1.set(false);}
    public void solenoidExtend() {m_singleSolenoid0.set(true);
                                  m_singleSolenoid1.set(false);}
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
        return runEnd(() -> {spindexter.set(ControlMode.PercentOutput,-.30);}, 
        () -> spindexter.set(ControlMode.PercentOutput, 0.0));}
    public CommandBase spindexterCCW() {
        return runEnd(() -> {spindexter.set(ControlMode.PercentOutput,.30);},
        () -> spindexter.set(ControlMode.PercentOutput, 0.0));} 
    public void spindexterStop() {
        spindexter.set(ControlMode.PercentOutput, 0.0);
    }

}

