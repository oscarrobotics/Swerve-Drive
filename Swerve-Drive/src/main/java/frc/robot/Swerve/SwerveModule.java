package frc.robot.Swerve;

import com.revrobotics.AnalogInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;

public class SwerveModule {

    public final String moduleName;
    public final int moduleNumber;
    
    /* Motors/Encoders */

    private CANSparkMax m_driveMotor;
    private CANSparkMax m_steerMotor;

    private RelativeEncoder m_driveEncoder;
    private RelativeEncoder m_steerEncoder;

    private SparkMaxPIDController m_drivePIDController;
    private SparkMaxPIDController m_steerPIDController;

    /* Boolean conditions in case the positions are 
    reversed to how we want it when physically starting the bot */
    private boolean isAbsEncoderReversed;
    private double absoluteEncoderOffsetRad;

    



    //Constructor
    //pass in constants
    public SwerveModule(String moduleName, int moduleNumber, int driveCANId, int steerCANId, double angularOffset, boolean isReversed) {
        this.moduleName = moduleName;
        this.moduleNumber = moduleNumber;
        m_driveMotor = new CANSparkMax(driveCANId, MotorType.kBrushless);
        m_steerMotor = new CANSparkMax(steerCANId, MotorType.kBrushless);

        m_driveEncoder = m_driveMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
        m_steerEncoder = m_steerMotor.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);


        m_drivePIDController = m_driveMotor.getPIDController();
        m_steerPIDController = m_steerMotor.getPIDController();
        
        angularOffset = 0.0;
        //Important to know the motor's states before configuration; also useful for first 
        //setting up the motors
        m_driveMotor.restoreFactoryDefaults();
        m_steerMotor.restoreFactoryDefaults();

        resetEncoders();

    }
    //Controller --> Chassispeeds --> Swerve Module State [x4]

    //get abs angle


    //optimize turning
    public SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle){
        double targetAngle = orientTo2Pi(currentAngle.getDegrees(), desiredState.angle.getDegrees());
        double targetSpeed = desiredState.speedMetersPerSecond;
        double delta = targetAngle - currentAngle.getDegrees();
        if(Math.abs(delta) > 90){
            targetSpeed = -targetSpeed;
            targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
        }
        return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
    }


    //orient angle
    public double orientTo2Pi(double curAngle, double newAngle){
        double lowerBound;
        double upperBound;
        double lowerOffset = curAngle % 360;

        //Whether the angle is above a full rotation
        if(lowerOffset >= 0){
            lowerBound = curAngle - lowerOffset;
            upperBound = curAngle + (360 - lowerOffset);
        } else {
            upperBound = curAngle - lowerOffset;
            lowerBound = curAngle + (360 + lowerOffset);
        }
        while(newAngle < lowerBound){
            newAngle += 360;
        }
        while(newAngle > upperBound){
            newAngle -= 360;
        }
        if(newAngle - curAngle > 180){
            newAngle += 360;
        }
        return newAngle;
    }

    //set position and turning angle
    public void setAngle(SwerveModuleState desiredState, boolean steerInPlace){

    }
    //config motor offset (in the case the motor and encoder are not aligned)

    // private Rotation2d getAngle(){
        
    // }

    //Zero out motors
    public void resetEncoders(){
        m_driveEncoder.setPosition(0);
    }

    //Subject to change if we find that accounting for the Absolute Encoder's angle
    //at all times is preferable to the internal encoder's
    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            m_driveEncoder.getPosition(), new Rotation2d(getTurningPosition()));
    }
   
    public double getTurningPosition(){ 
        return m_steerEncoder.getPosition();
    }

    public double getDriveVelocity(){
        return m_driveEncoder.getVelocity();
    }

    public double getTurningVelocity(){
        return m_steerEncoder.getVelocity();
    }

    //Get Distance Command

    // /* (Given this is an analog input), you can divide the encoder's voltage reading by 
    //  * the amount supplied to obtain its percent of a full rotation.
    //  * 
    //  * Simply multiply by 2pi and subtract the offset to obtain the current angle
    //  * in radians.
    //  */
    // public double getAbsoluteEncoderRad(){
    //     double theta = m_absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
    //     theta *= (2.0 * Math.PI);
    //     theta -= absoluteEncoderOffsetRad;
    //     return theta * (isAbsEncoderReversed ? -1.0 : 1.0);

    // }

       //Set desired state (based on utilizing the SparkMAX PID Controllers)
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop, boolean steerInPlace){
        desiredState = optimize(desiredState, getState().angle);
        // m_driveMotor.set(desiredState.speedMetersPerSecond / kPhysicalMaxSpeedMetersPerSecond);

        m_drivePIDController.setReference(desiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
        m_steerPIDController.setReference(desiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

    }

    
     //m/s to rpm
     //Velocity(m/s) / wheel circumference (meters/revolution) * gear ratio (unitless) * 60 (seconds) --> dimensional analysis
    // private double VelocitytoRPM(double velocityMetersPerSecond){

    //     return (velocityMetersPerSecond * 60 * kGearRatio) / ((Units.inchesToMeters(kWheelDiameterInches) * Math.PI));
 
    // }

    public void stop(){
        m_driveMotor.set(0);
        m_steerMotor.set(0);
    }
}
