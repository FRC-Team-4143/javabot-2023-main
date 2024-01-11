package frc.robot.commands4143;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer4143;
import frc.robot.Constants.gamePiece;
import frc.robot.subsystems4143.Arm;
import frc.robot.subsystems4143.CubeSubsystem;
import frc.robot.subsystems4143.PickupSubsystem;

public class CubeOut extends Command{
    private CubeSubsystem cubeSubsystem;
    private int count;
    private Arm arm;
    public RobotContainer4143 container;

    public CubeOut(CubeSubsystem cubeSubsystem, Arm arm, RobotContainer4143 container){
        this.container = container;
        this.cubeSubsystem = cubeSubsystem;
        this.arm =arm;
        count=0;
        
    }

    @Override
    public void initialize() {
        container.m_led.setData(container.m_ledBufferCube);
        container.currentMode = Constants.gamePiece.Cube;
        count = 0;
        //arm.elevatorPickup();
        addRequirements(cubeSubsystem);
        cubeSubsystem.rackOut();
        container.getArm().setHomePosition(container).schedule();
    }

    @Override
    public void execute() {
            cubeSubsystem.rollersSet(-1);

        count+=1;

    }

    @Override
    public void end(boolean interrupted){
        if(!DriverStation.isAutonomous()){
            cubeSubsystem.rackIn();
            cubeSubsystem.rollersSet(0.0);
        }

    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
