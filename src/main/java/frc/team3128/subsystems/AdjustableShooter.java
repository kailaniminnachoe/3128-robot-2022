package frc.team3128.subsystems;

import frc.team3128.Constants.AdjustableShooterConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3128.common.hardware.motorcontroller.NAR_CANSparkMax;
import frc.team3128.common.infrastructure.NAR_PIDSubsystem;
import net.thefletcher.revrobotics.enums.MotorType;

public class AdjustableShooter extends NAR_PIDSubsystem{

    //defintions 
    private static AdjustableShooter instance;
    private NAR_CANSparkMax m_hoodShooter;
    private DigitalInput m_hoodSensor;
    private double desiredAngle; 

    private double previousTime, currentTime; // seconds
    private double previousError, currentError;
    private boolean isAdjusted;
    private double txThreshold = AdjustableShooterConstants.TX_THRESHOLD; //degrees
    private int plateauCount;

    //configuration    
    public AdjustableShooter() {
        super(new PIDController(
            AdjustableShooterConstants.ADJUSTABLE_SHOOTER_PID_kP, 
            AdjustableShooterConstants.ADJUSTABLE_SHOOTER_PID_kI, 
            AdjustableShooterConstants.ADJUSTABLE_SHOOTER_PID_kD), 
            AdjustableShooterConstants.PLATEAU_COUNT);

        configMotors();
        configSensors();
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
        m_hoodShooter = new NAR_CANSparkMax(AdjustableShooterConstants.HOOD_SHOOTER_ID, MotorType.kBrushless);
    }

    //sensors
    private void configSensors() {
        m_hoodSensor= new DigitalInput(AdjustableShooterConstants.HOOD_SENSOR_ID);
    }

    //Similar to Shooter PID, however does not have a "state" but uses angle 
    public void beginAdjust(double desiredAngle) {
        desiredAngle = this.desiredAngle; 
        super.setSetpoint(desiredAngle);
    }

    public void stopAdjust() {
        super.resetPlateauCount();
        setSetpoint(0);
    }

    @Override
    protected double getMeasurement() {
        return m_hoodShooter.getSelectedSensorPosition();
    } 

    /**
     * Use the raw voltage output from the PID loop, add a feed forward component, and convert it to a percentage of total
     * possible voltage to apply to the motors.
     * Do we want to implent the sensors or are already implemented in this PID loop? 
     * 
     * @param output Output from the PID Loop (RPM)
     * @param setpoint The desired setpoint RPM for the PID Loop (RPM)
     */

    @Override
    protected void useOutput(double currentAngle, double desiredAngle) {
        desiredAngle = this.desiredAngle; 
        currentAngle = getMeasurement(); 
        currentTime = RobotController.getFPGATime()/1e6; 
        currentError = desiredAngle - currentAngle;

        if (txThreshold < AdjustableShooterConstants.TX_THRESHOLD_MAX) {
            txThreshold += (currentTime - previousTime) * (AdjustableShooterConstants.TX_THRESHOLD_INCREMENT);
        }

        double ff = Math.signum(currentError) * AdjustableShooterConstants.ADJUSTABLE_SHOOTER_PID_kF;
        double feedbackPower = AdjustableShooterConstants.ADJUSTABLE_SHOOTER_PID_kP * currentError + AdjustableShooterConstants.ADJUSTABLE_SHOOTER_PID_kD * (currentError - previousError) / (currentTime - previousTime) + ff;
                
        m_hoodShooter.set(feedbackPower); 

        if (Math.abs(currentError) < txThreshold) {
            plateauCount++;
            if (plateauCount > AdjustableShooterConstants.ALIGN_PLATEAU_COUNT) {
                isAdjusted = true;
                }
        else {
            isAdjusted = false;
            plateauCount = 0;
            }
        }   
        previousError = currentError; 
        
    }

     public void periodic() {
        //Narwhal dashboard
        SmartDashboard.putString("HoodAngle", m_hoodSensor.toString());
        SmartDashboard.putBoolean("isAdjusted", isAdjusted);
    }

    public double calculateMotorVelocityFromDist(double angle) {
        return 0; //math ask someone 
        //would this be getMeasuremnet? 
    }
}