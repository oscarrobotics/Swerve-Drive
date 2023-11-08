package frc.robot.Swerve;

import com.revrobotics.AnalogInput;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.math.util.Units;

public class SwerveModule {
    
    /* Motors/Encoders */

    private CANSparkMax m_driveMotor;
    private CANSparkMax m_steerMotor;

    private SparkMaxRelativeEncoder m_driveEncoder;
    private SparkMaxRelativeEncoder m_turningEncoder;

    private SparkMaxPIDController turningPIDController;

    //Assuming the absolute encoder is attached to the RIO--otherwise 
    //it may just be the SparkMaxAbsoluteEncoder class if built in
    private AnalogInput absoluteEncoder;

    /* Boolean conditions in case the positions are 
    reversed to how we want it when physically starting the bot */
    private boolean isAbsEncoderReversed;
    private double absoluteEncoderOffsetRad;

    

    /* Constants (may move later) */

    final double kGearRatio = 4.71; //4.71:1 ???
    final int kWheelDiameterInches = 3;

    //Controller --> Chassispeeds --> Swerve Module State [x4]

    //get abs angle

    //Set desired state
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop, boolean steerInPlace){



    }

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
    public double orientTo2Pi(double scopeReference, double newAngle){
        double lowerBound;
        double upperBound;
        double lowerOffset = scopeReference % 360;

        //Whether the angle is above a full rotation
        if(lowerOffset >= 0){
            lowerBound = scopeReference - lowerOffset;
            upperBound = scopeReference + (360 - lowerOffset);
        } else {
            upperBound = scopeReference - lowerOffset;
            lowerBound = scopeReference + (360 + lowerOffset);
        }
        while(newAngle < lowerBound){
            newAngle += 360;
        }
        while(newAngle > upperBound){
            newAngle -= 360;
        }
        if(newAngle - scopeReference > 180){
            newAngle += 360;
        }
        return newAngle;
    }

    //setters + getters for speed + theta (separately)

    //get drive motor direction

    //get motor position

    //config motor offset (in the case the motor and encoder are not aligned)

    //zero out drive motor


    /* Gear ratio is 4.71:1 according to the MAXSwerve Module
     purchashing page */

    public SwerveModuleState getState(){
        return new SwerveModuleState();
    }

    public double getDrivePosition(){ 
        return 0.0;
    }
    public double getTurningPosition(){ 
        return 0.0;
    }

    public double getDriveVelocity(){ return 0.0;}
    public double getTurningVelocity(){ return 0.0;}


     //m/s to rpm
      //Velocity(m/s) / wheel circumference (meters/revolution) * gear ratio (unitless) * 60 (seconds) --> dimensional analysis
    private double VelocitytoRPM(double velocityMetersPerSecond){

        return (velocityMetersPerSecond * 60 * kGearRatio) / ((Units.inchesToMeters(kWheelDiameterInches) * Math.PI));
 
    }

}
