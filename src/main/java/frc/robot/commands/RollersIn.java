package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class RollersIn extends CommandBase{
    private Arm arm;
    
    public RollersIn(Arm arm){
        this.arm = arm;
    }

    @Override
    public void execute() {
        arm.rollersin();
    }

    @Override
    public void end(boolean interrupted){
        arm.rollerstop();
    }
}
