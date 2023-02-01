package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class RollersOut extends CommandBase{
    private Arm arm;
    
    public RollersOut(Arm arm){
        this.arm = arm;
    }

    @Override
    public void execute() {
        arm.rollersout();
    }

    @Override
    public void end(boolean interrupted){
        arm.rollerstop();
    }
}
