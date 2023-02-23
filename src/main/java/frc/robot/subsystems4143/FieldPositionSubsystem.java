package frc.robot.subsystems4143;

import frc.lib.controller.Axis;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FieldPositionSubsystem extends SubsystemBase{
    private int xPosition = 1;
    private int yPosition = 1;
    private int count = 0;
    public FieldPositionSubsystem(){
    }

    public int getXPosition() {
        return xPosition;
    }

    public int getYPosition() {
        return yPosition;
    }

    public Command scoreSelectCommand(Axis forward, Axis strafe) {
        return run(() -> {
            if(count > 0) count -= 1;
            if(count == 0) {
                if(forward.get() > .5 && xPosition < 3) { xPosition+=1; count = 10;}
                if(forward.get() < -.5 && xPosition > 1) { xPosition-=1; count = 10;}
                if(strafe.get() > .5 && yPosition < 9) { yPosition+=1; count = 10;}
                if(strafe.get() < -.5 && yPosition > 1) { yPosition-=1; count = 10;}
            }
           }).ignoringDisable(true);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("xPosition", xPosition);
        SmartDashboard.putNumber("yPosition", yPosition);
        SmartDashboard.putBoolean("loc1,1", xPosition == 1 && yPosition == 1);
        SmartDashboard.putBoolean("loc1,2", xPosition == 1 && yPosition == 2);
        SmartDashboard.putBoolean("loc1,3", xPosition == 1 && yPosition == 3);
        SmartDashboard.putBoolean("loc1,4", xPosition == 1 && yPosition == 4);
        SmartDashboard.putBoolean("loc1,5", xPosition == 1 && yPosition == 5);
        SmartDashboard.putBoolean("loc1,6", xPosition == 1 && yPosition == 6);
        SmartDashboard.putBoolean("loc1,7", xPosition == 1 && yPosition == 7);
        SmartDashboard.putBoolean("loc1,8", xPosition == 1 && yPosition == 8);
        SmartDashboard.putBoolean("loc1,9", xPosition == 1 && yPosition == 9);
        SmartDashboard.putBoolean("loc2,1", xPosition == 2 && yPosition == 1);
        SmartDashboard.putBoolean("loc2,2", xPosition == 2 && yPosition == 2);
        SmartDashboard.putBoolean("loc2,3", xPosition == 2 && yPosition == 3);
        SmartDashboard.putBoolean("loc2,4", xPosition == 2 && yPosition == 4);
        SmartDashboard.putBoolean("loc2,5", xPosition == 2 && yPosition == 5);
        SmartDashboard.putBoolean("loc2,6", xPosition == 2 && yPosition == 6);
        SmartDashboard.putBoolean("loc2,7", xPosition == 2 && yPosition == 7);
        SmartDashboard.putBoolean("loc2,8", xPosition == 2 && yPosition == 8);
        SmartDashboard.putBoolean("loc2,9", xPosition == 2 && yPosition == 9);
        SmartDashboard.putBoolean("loc3,1", xPosition == 3 && yPosition == 1);
        SmartDashboard.putBoolean("loc3,2", xPosition == 3 && yPosition == 2);
        SmartDashboard.putBoolean("loc3,3", xPosition == 3 && yPosition == 3);
        SmartDashboard.putBoolean("loc3,4", xPosition == 3 && yPosition == 4);
        SmartDashboard.putBoolean("loc3,5", xPosition == 3 && yPosition == 5);
        SmartDashboard.putBoolean("loc3,6", xPosition == 3 && yPosition == 6);
        SmartDashboard.putBoolean("loc3,7", xPosition == 3 && yPosition == 7);
        SmartDashboard.putBoolean("loc3,8", xPosition == 3 && yPosition == 8);
        SmartDashboard.putBoolean("loc3,9", xPosition == 3 && yPosition == 9);
    }
}
