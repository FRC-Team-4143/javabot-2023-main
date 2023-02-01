package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class Spindexter extends CommandBase{
    private Arm arm;
    
    public Spindexter(Arm arm){
        this.arm = arm;
    }

    @Override
    public void execute() {
        arm.spindexterCW();
    }

    @Override
    public void end(boolean interrupted){
        arm.spindexterStop();
    }
}
