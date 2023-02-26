package frc.robot.commands4143;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems4143.Arm;

public class ElevatorDown extends CommandBase{
    private Arm arm;
    
    public ElevatorDown(Arm arm){
        this.arm = arm;
    }

    @Override
    public void execute() {
        arm.elevatorMove(0.02);
    }

    @Override
    public void end(boolean interrupted){
        arm.elevatorMove(0);
    }
}