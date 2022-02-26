package frc.team3128.commands;

import frc.team3128.subsystems.AdjustableShooter;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CmdAdjust extends CommandBase{
    private AdjustableShooter adjustableShooter;
    private double desiredAngle;
    
    public CmdAdjust(AdjustableShooter adjustableShooter, double desiredAngle) {
        
        this.adjustableShooter = adjustableShooter;
        this.desiredAngle = desiredAngle;

        addRequirements(adjustableShooter, desiredAngle);
    }
    
    private void addRequirements(AdjustableShooter adjustableShooter2, double desiredAngle) {
    }

    @Override
    public void initialize() {
        adjustableShooter.beginAdjust(desiredAngle);
    }
    
    @Override
    public void end(boolean interrupted) {
        adjustableShooter.stopAdjust();
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}
