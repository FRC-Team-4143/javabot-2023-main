package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ElevatorUp extends CommandBase{
    private Arm arm;
    
    public ElevatorUp(Arm arm){
        this.arm = arm;
    }

    @Override
    public void execute() {
        arm.setElevatorSpeed(-0.25);
        System.out.println("it's working");
    }

    @Override
    public void end(boolean interrupted){
        arm.setElevatorSpeed(0);
    }
}