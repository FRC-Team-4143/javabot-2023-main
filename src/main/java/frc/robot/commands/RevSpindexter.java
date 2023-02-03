package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class RevSpindexter extends CommandBase{
    private Arm arm;
    
    public RevSpindexter(Arm arm){
        this.arm = arm;
    }

    @Override
    public void execute() {
        arm.spindexterCCW();
    }

    @Override
    public void end(boolean interrupted){
        arm.spindexterStop();
    }
}
