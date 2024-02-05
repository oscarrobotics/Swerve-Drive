package frc.robot.Swerve;

import java.lang.reflect.Array;
import java.util.Arrays;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.stream.Collector;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveSubsystem extends SubsystemBase{

    private final Pigeon2 m_gyro = new Pigeon2(25);

    // private SwerveDriveOdometry swerveOdometry;
    private SwerveDriveKinematics m_kinematics;
    // private SwerveDriveState m_state;
    private Field2d m_field;
    public Matrix<N3,N1> stateStdDevs = VecBuilder.fill(0.1,0.1,0.1); //values uncertain
    public Matrix<N3,N1> visionMeasurementStdDevs = VecBuilder.fill(0.9,0.9,0.9); //values uncertain



    private final SwerveModule[] m_modules = new SwerveModule[]{
            new SwerveModule("Rear Left", 0, SwerveConstants.Mod0.constants),
            new SwerveModule("Front Left", 1, SwerveConstants.Mod1.constants),
            new SwerveModule("Rear Right", 2,  SwerveConstants.Mod2.constants),
            new SwerveModule("Front Right", 3, SwerveConstants.Mod3.constants),
    };
    public SwerveDrivePoseEstimator m_poseEstimator ;



    public SwerveSubsystem(){

        
        m_field = new Field2d();
        m_kinematics = new SwerveDriveKinematics(
            //garentees the order of positional offsets is the order of m_modulesu 
            Arrays.stream(m_modules).map(mod -> mod.positionalOffset).toArray(Translation2d[]::new)
        );
        m_poseEstimator = new SwerveDrivePoseEstimator(
                    m_kinematics, 
                    Rotation2d.fromDegrees(m_gyro.getAngle()) , 
                    getModulePositions(),
                    new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(180)), 
                    stateStdDevs,
                    visionMeasurementStdDevs);
        
        SmartDashboard.putData("Field", m_field);

        var pigeon2YawSignal = m_gyro.getYaw();
    }

    public void drive(double vxMeters, double vyMeters, double omegaRadians, boolean fieldRelative, boolean isOpenLoop){
 
        ChassisSpeeds targetChassisSpeeds = fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(vxMeters, vyMeters, omegaRadians, getHeading())
            : new ChassisSpeeds(vxMeters, vyMeters, omegaRadians);

        setChassisSpeeds(targetChassisSpeeds, isOpenLoop, false);
    }
    public void drive(double vxMeters, double vyMeters, double omegaRadians, boolean fieldRelative, boolean isOpenLoop, Translation2d turnCenter){
 
        ChassisSpeeds targetChassisSpeeds = fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(vxMeters, vyMeters, omegaRadians, getHeading())
            : new ChassisSpeeds(vxMeters, vyMeters, omegaRadians);

        setChassisSpeeds(targetChassisSpeeds, isOpenLoop, false, turnCenter);
    }

    public void setChassisSpeeds(ChassisSpeeds targetChassisSpeeds, boolean openLoop, boolean steerInPlace){
        setModuleStates(m_kinematics.toSwerveModuleStates(targetChassisSpeeds), openLoop, steerInPlace);
    }
    public void setChassisSpeeds(ChassisSpeeds targetChassisSpeeds, boolean openLoop, boolean steerInPlace, Translation2d turnCenter){
        setModuleStates(m_kinematics.toSwerveModuleStates(targetChassisSpeeds, turnCenter), openLoop, steerInPlace);
    }

    public SwerveModuleState[] getModuleStates() {
		SwerveModuleState[] states = new SwerveModuleState[4];
		for (SwerveModule mod : m_modules) {
			states[mod.moduleNumber] = mod.getState();
		}
		return states;
	}

    public SwerveModulePosition[] getModulePositions() {
		SwerveModulePosition[] positions = new SwerveModulePosition[4];
		for (SwerveModule mod : m_modules) {
			positions[mod.moduleNumber] = mod.getPosition();
		}
		return positions;
	}

    public void setModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop, boolean steerInPlace){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, 14);

        for (SwerveModule mod : m_modules){
            mod.setDesiredState(desiredStates[mod.moduleNumber], isOpenLoop, steerInPlace);
            SmartDashboard.putNumber(mod.moduleName+"requested Angle", desiredStates[mod.moduleNumber].angle.getRadians());
        }
    }

    public Rotation2d getHeading(){
        return m_poseEstimator.getEstimatedPosition().getRotation();
    }

    private StatusSignal<Double> getGyroYaw(){
        return m_gyro.getYaw();
    }

    public Command teleopDrive(
            DoubleSupplier translation, DoubleSupplier strafe, DoubleSupplier rotation,
            BooleanSupplier fieldRelative, BooleanSupplier openLoop){
        return run(() -> {
            double translationVal = MathUtil.applyDeadband(translation.getAsDouble(), Constants.swerveDeadband);
            double strafeVal = MathUtil.applyDeadband(strafe.getAsDouble(), Constants.swerveDeadband);
            double rotationVal = MathUtil.applyDeadband(rotation.getAsDouble(), Constants.swerveDeadband);

            
            
            boolean isOpenLoop = openLoop.getAsBoolean();

            translationVal *= Constants.kPhysicalMaxSpeedMetersPerSecond;

            strafeVal *= Constants.kPhysicalMaxSpeedMetersPerSecond;

            rotationVal *= Constants.kMaxRotSpeedRadPerSecond;
            drive(translationVal, strafeVal, rotationVal, fieldRelative.getAsBoolean(), isOpenLoop);
        }).withName("Teleop Drive");
    }
    public Command evasiveDrive(
            DoubleSupplier translation, DoubleSupplier strafe, DoubleSupplier rotation,
            BooleanSupplier fieldRelative, BooleanSupplier openLoop){
        return run(() -> {
            double translationVal = MathUtil.applyDeadband(translation.getAsDouble(), Constants.swerveDeadband);
            double strafeVal = MathUtil.applyDeadband(strafe.getAsDouble(), Constants.swerveDeadband);
            double rotationVal = MathUtil.applyDeadband(rotation.getAsDouble(), Constants.swerveDeadband);

            //calculate evasive center
            //max turninc center = over modual
            //turning center = strafe vector * 23in
            // Translation2d turningCenter = new Translation2d(translationVal*0.5969/2,strafeVal*0.5969/2);
            Translation2d turningCenter = new Translation2d(translationVal*0.5969*2,strafeVal*0.5969*2);
            
            
            boolean isOpenLoop = openLoop.getAsBoolean();

            translationVal *= Constants.kPhysicalMaxSpeedMetersPerSecond;

            strafeVal *= Constants.kPhysicalMaxSpeedMetersPerSecond;

            rotationVal *= Constants.kMaxRotSpeedRadPerSecond;

            //calculate evasive center
            //max turninc center = over modual
            


            drive(translationVal, strafeVal, rotationVal, fieldRelative.getAsBoolean(), isOpenLoop, turningCenter);
        }).withName("Evasive Drive");
    }

    public void resetOdometry(){
        m_poseEstimator.resetPosition(Rotation2d.fromDegrees(m_gyro.getAngle()), 
        getModulePositions(), 
        new Pose2d(new Translation2d(0,0), 
        Rotation2d.fromDegrees(180)));

    }

    public void updateOdometry(){
        m_poseEstimator.update(m_gyro.getRotation2d(), getModulePositions());
        
    }

    @Override
    public void periodic(){
        updateOdometry();
        for(SwerveModule mod : m_modules){
            // SmartDashboard.putNumber(mod.moduleName + "Desired Angle", mod.getAbsoluteAngle().getRadians());
            // SmartDashboard.putNumber(mod.moduleName +"Desired Velocity", mod.getDriveVelocity());
            
            SmartDashboard.putNumber(mod.moduleName +"Angle", mod.getAngle());
            
        }
    }
}


