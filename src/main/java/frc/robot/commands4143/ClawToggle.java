package frc.robot.commands4143;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems4143.Arm;

public class ClawToggle extends CommandBase{
    private Arm arm;
    private int count;

    public ClawToggle(Arm arm){
        this.arm = arm;
        count=0;
    }

    @Override
    public void initialize() {
        count = 0;
        
    }

    @Override
    public void execute() {
        if (ArmConstants.isClawOpen) arm.setClawSpeed(-0.5);
        else arm.setClawSpeed(0.5);
        count+=1;

    }

    @Override
    public void end(boolean interrupted){
        if(ArmConstants.isClawOpen){
            arm.setClawSpeed(-0.1);
        }
        else {
            arm.setClawSpeed(0);
        }
        ArmConstants.isClawOpen = !ArmConstants.isClawOpen;
    }

    @Override
    public boolean isFinished(){
        if (count > 50) {
            return true;
        }
        return false;
    }
}
