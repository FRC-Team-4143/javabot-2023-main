package frc.robot.commands4143;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems4143.Arm;

public class RotateDown extends CommandBase{
    private Arm arm;
    
    public RotateDown(Arm arm){
        this.arm = arm;
    }

    @Override
    public void execute() {
        arm.setRotate(-0.5);
    }

    @Override
    public void end(boolean interrupted){
        arm.setRotate(0);
    }
}