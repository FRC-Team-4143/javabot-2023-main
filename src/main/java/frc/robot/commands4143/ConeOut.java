package frc.robot.commands4143;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer4143;
import frc.robot.subsystems4143.ConeSubsystem;
import frc.robot.Constants;

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
        container.m_led.setData(container.m_ledBufferCone);
        container.currentMode = Constants.gamePiece.Cone;
        container.getArm().setHybridPosition(container).beforeStarting(new WaitCommand(.25)).schedule();
        //container.getArm().setHybrid();
        //coneSubsystem.intermediatePickup();
    }

    @Override
    public void execute() {
        coneSubsystem.setAngle(-168);
        coneSubsystem.setPickupMotorSpeed(-1);
    }

    @Override
    public void end(boolean interrupted){
        coneSubsystem.setAngle(-100);//-107 at tremont
        coneSubsystem.setPickupMotorSpeed(0);
        container.getArm().setClawClosed(container).beforeStarting(new WaitCommand(0.5)).schedule();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
