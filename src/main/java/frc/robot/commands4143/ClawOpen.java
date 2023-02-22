package frc.robot.commands4143;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems4143.Arm;

public class ClawOpen extends CommandBase{
    private Arm arm;
    
    public ClawOpen(Arm arm){
        this.arm = arm;
    }

    @Override
    public void execute() {
        arm.setClawSpeed(0.25);
    }

    @Override
    public void end(boolean interrupted){
        arm.setClawSpeed(0);
    }
}
