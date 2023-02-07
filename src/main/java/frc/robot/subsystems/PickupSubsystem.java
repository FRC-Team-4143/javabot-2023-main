package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PickupSubsystem extends SubsystemBase {
    private DoubleSolenoid m_doubleSolenoid;
    private TalonFX toproller;
    private TalonFX bottomroller;
    private VictorSPX spindexter; 


    public void PickupSubsystem () {
        spindexter = new VictorSPX(10);
        toproller = new TalonFX(25);
        bottomroller = new TalonFX(26);

        m_doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 3, 2);
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

}

