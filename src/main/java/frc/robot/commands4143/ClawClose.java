package frc.robot.commands4143;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer4143;
import frc.robot.subsystems4143.Arm;
import frc.robot.Constants.gamePiece;

public class ClawClose extends CommandBase{
    private Arm arm;
    private int count;
    private RobotContainer4143 container;

    public ClawClose(Arm arm, RobotContainer4143 container){
        this.container = container;
        this.arm = arm;
        count=0;
    }

    @Override
    public void initialize() {
        count = 0;
    }

    @Override
    public void execute() {
        if(count == 30 && arm.getDistance() < -0.29){
            arm.setRotate(-5);
        } else if(count < 50){
            if (container.currentMode == gamePiece.Cone) {
                arm.setClawSpeed(-0.75);
            }
            else if (container.currentMode == gamePiece.Cube){
                arm.setClawSpeed(-0.25);
            }
        } else {
            arm.setClawSpeed(-0.25);
        }
        count+=1;

    }

    @Override
    public void end(boolean interrupted){
        
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
