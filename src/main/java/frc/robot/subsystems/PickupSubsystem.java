package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

public class PickupSubsystem extends SubsystemBase {
    private DoubleSolenoid m_doubleSolenoid;
    private TalonFX toproller;
    private TalonFX bottomroller;
    private VictorSPX spindexter; 


    public PickupSubsystem () {
        spindexter = new VictorSPX(10);
        toproller = new TalonFX(25);
        bottomroller = new TalonFX(26);

        m_doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 3, 2);
    }

    //public void pickupRetract() {m_doubleSolenoid.set(DoubleSolenoid.Value.kForward);}
    //public void pickupExtend() {m_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);}
    //public void pickupOff() {m_doubleSolenoid.set(DoubleSolenoid.Value.kOff);}

    public CommandBase pickupExtend() {
        return new FunctionalCommand (
            () -> {m_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);}, () -> {},
            interrupted ->  {m_doubleSolenoid.set(DoubleSolenoid.Value.kOff);}, () -> false);
    }

    public CommandBase pickupRetract() {
        return new FunctionalCommand(() -> {m_doubleSolenoid.set(DoubleSolenoid.Value.kForward);}, () -> {},interrupted ->  {}, () -> false);
    }


    public CommandBase rollIn() {
        return new FunctionalCommand(() -> {toproller.set(ControlMode.PercentOutput,-.40); 
                        bottomroller.set(ControlMode.PercentOutput,-.40);}, () -> {},
                      interrupted -> {toproller.set(ControlMode.PercentOutput,0); 
                        bottomroller.set(ControlMode.PercentOutput,0);}, () -> false
        );
    }

    public CommandBase rollOut() {
        return new FunctionalCommand(() -> {toproller.set(ControlMode.PercentOutput,.40); 
                        bottomroller.set(ControlMode.PercentOutput,.40);}, () -> {},
                      interrupted -> {toproller.set(ControlMode.PercentOutput,0); 
                        bottomroller.set(ControlMode.PercentOutput,0);}, () -> false
        );
    }

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

