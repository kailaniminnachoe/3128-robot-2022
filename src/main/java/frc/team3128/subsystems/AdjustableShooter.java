package frc.team3128.subsystems;

import frc.team3128.Constants.AdjustableShooterConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3128.common.hardware.motorcontroller.NAR_CANSparkMax;
import frc.team3128.common.infrastructure.NAR_PIDSubsystem;
import frc.team3128.common.narwhaldashboard.NarwhalDashboard;
import net.thefletcher.revrobotics.enums.MotorType;

public class AdjustableShooter extends NAR_PIDSubsystem{

    //defintions 
    private static AdjustableShooter instance;
    private NAR_CANSparkMax m_adjustableShooter;
    private double currentAngle; 
    private double desiredAngle = AdjustableShooterConstants.SETPOINT_ANGLE; 

    private double previousTime = 0, currentTime = 0; // seconds
    private double previousError, currentError;
    private boolean isAdjusted;
    private double thresholdPercent = AdjustableShooterConstants.ADJUSTABLE_SHOOTER_THRESHOLD; //degrees
    private int plateauCount = 0;

    //configuration    
    public AdjustableShooter() { 
        super(new PIDController( 
            AdjustableShooterConstants.ADJUSTABLE_SHOOTER_PID_kP, 
            AdjustableShooterConstants.ADJUSTABLE_SHOOTER_PID_kI, 
            AdjustableShooterConstants.ADJUSTABLE_SHOOTER_PID_kD), 
            AdjustableShooterConstants.PLATEAU_COUNT);

        configMotors();
    }

    //instance
    public static synchronized AdjustableShooter getInstance() {
        if(instance == null) {
            instance = new AdjustableShooter();
        }
        return instance;
    }

    //motor
    private void configMotors() {
        m_adjustableShooter = new NAR_CANSparkMax(AdjustableShooterConstants.ADJUSTABLE_SHOOTER_ID, MotorType.kBrushless);
    }

    //Similar to Shooter PID, however does not have a "state" but uses angle 
    //includes StartPID
    public void beginAdjust(double desiredAngle) {
        desiredAngle = AdjustableShooterConstants.SETPOINT_ANGLE; 
        super.setSetpoint(desiredAngle); //Angle calculated dependent on math; could be set to a degree to test
        super.resetPlateauCount();
    }

    public void stopAdjust() {
        super.resetPlateauCount();
        setSetpoint(0); //zeroes at base
    }

    @Override
    protected double getMeasurement() {
        return m_adjustableShooter.get(); //gets encoder count
    } 

    private double getAngle(double currentAngle) {
        currentAngle = getMeasurement(); 
        return currentAngle * 360 / AdjustableShooterConstants.ADJUSTABLE_SHOOTER_GEAR_RATIO; //degrees
    }

    private double getArcLengthFromAngle (double currentAngle) {
        currentAngle = getAngle(currentAngle);
        return AdjustableShooterConstants.ADJUSTABLE_MOTOR_TO_OUTER_PLATE_DISTANCE * currentAngle / 360; //inches
    }

    @Override
    protected void useOutput(double currentAngle, double desiredAngle) {
        currentAngle = getMeasurement(); 
        desiredAngle = this.desiredAngle; 
        currentTime = RobotController.getFPGATime()/1e6; 
        currentError = desiredAngle - currentAngle;

        currentTime = RobotController.getFPGATime() / 1e6;
        if (thresholdPercent < AdjustableShooterConstants.ADJUSTABLE_SHOOTER_THRESHOLD_MAX) {
            thresholdPercent += ((currentTime - previousTime) * ((AdjustableShooterConstants.ADJUSTABLE_SHOOTER_THRESHOLD_MAX - AdjustableShooterConstants.ADJUSTABLE_SHOOTER_THRESHOLD_INCREMENT)) / AdjustableShooterConstants.TIME_TO_MAX_THRESHOLD);
            getController().setTolerance(thresholdPercent * desiredAngle);
        }

        checkPlateau(desiredAngle, AdjustableShooterConstants.ADJUSTABLE_SHOOTER_THRESHOLD_INCREMENT);

        double ff = Math.signum(currentError) * AdjustableShooterConstants.ADJUSTABLE_SHOOTER_PID_kF;
        double feedbackPower = AdjustableShooterConstants.ADJUSTABLE_SHOOTER_PID_kP * currentError + AdjustableShooterConstants.ADJUSTABLE_SHOOTER_PID_kD * (currentError - previousError) / (currentTime - previousTime) + ff;

        m_adjustableShooter.set(feedbackPower); 

        previousError = currentError; 
        previousTime = currentTime;
    }

     public void periodic() {
        //Narwhal dashboard
        NarwhalDashboard.put("HoodAngle", getAngle(currentAngle)); //number //maybe have ontop of graph with interpolation
        NarwhalDashboard.put("isAdjusted", isAdjusted);
    }
}