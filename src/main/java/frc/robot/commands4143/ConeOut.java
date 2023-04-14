package frc.robot.commands4143;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer4143;
import frc.robot.subsystems4143.ConeSubsystem;

public class ConeOut extends CommandBase{
    private ConeSubsystem coneSubsystem;
    public RobotContainer4143 container;

    public ConeOut(ConeSubsystem coneSubsystem, RobotContainer4143 container){
        this.container = container;
        this.coneSubsystem = coneSubsystem;
    }

    @Override
    public void initialize() {
    
        //arm.elevatorPickup();
        addRequirements(coneSubsystem);
        //coneSubsystem.intermediatePickup();
    }

    @Override
    public void execute() {
        coneSubsystem.setAngle(-168);
        coneSubsystem.setPickupMotorSpeed(-1);
    }

    @Override
    public void end(boolean interrupted){
        coneSubsystem.setAngle(-107);
        coneSubsystem.setPickupMotorSpeed(0);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
