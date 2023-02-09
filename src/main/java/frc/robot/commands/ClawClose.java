package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ClawClose extends CommandBase{
    private Arm arm;
    private int count;

    public ClawClose(Arm arm){
        this.arm = arm;
        count=0;
    }

    @Override
    public void initialize() {
        count = 0;
    }

    @Override
    public void execute() {
        arm.setClawSpeed(-0.5);
        count+=1;

    }

    @Override
    public void end(boolean interrupted){
        //arm.setClawSpeed(0);
        if(count<50){
            arm.setClawSpeed(0);
        }
        else {
            arm.setClawSpeed(-0.25);
        }
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
