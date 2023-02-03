package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class PickUpExtend extends CommandBase{
    private Arm arm;
    
    public PickUpExtend(Arm arm){
        this.arm = arm;
    }

    @Override
    public void execute() {
        arm.pickupExtend();
    }

    @Override
    public void end(boolean interrupted){
        arm.pickupOff();
    }
}
