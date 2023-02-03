package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ElevatorToPosition extends CommandBase{
    private Arm arm;
    
    public ElevatorToPosition(Arm arm){
        this.arm = arm;
    }

    @Override
    public void execute() {
        arm.elevatorMove(0.5);
    }

    @Override
    public void end(boolean interrupted){
        arm.elevatorMove(0);
    }
}