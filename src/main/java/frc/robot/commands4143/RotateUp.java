package frc.robot.commands4143;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems4143.Arm;

public class RotateUp extends CommandBase{
    private Arm arm;
    
    public RotateUp(Arm arm){
        this.arm = arm;
    }

    @Override
    public void execute() {
        arm.setRotateSpeed(1.0);
    }

    @Override
    public void end(boolean interrupted){
        arm.setRotateSpeed(0);
    }
}