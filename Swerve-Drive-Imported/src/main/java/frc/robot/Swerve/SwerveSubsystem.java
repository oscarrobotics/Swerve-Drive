package frc.robot.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveKinematics;

public class SwerveSubsystem extends SubsystemBase{

    private final Pigeon2 m_gyro = new Pigeon2(0);

    private SwerveDriveOdometry swerveOdometry;
    private SwerveModule[] m_modules;

    private Field2d m_field;

    public final SwerveModule flModule = new SwerveModule("Front Left", 0, SwerveConstants.Mod0.constants);
    public final SwerveModule frModule = new SwerveModule("Front Right", 1, SwerveConstants.Mod1.constants);
    public final SwerveModule rlModule = new SwerveModule("Rear Left", 2, SwerveConstants.Mod2.constants);
    public final SwerveModule rrModule = new SwerveModule("Rear Right", 3,  SwerveConstants.Mod3.constants);

    // public final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(m_kinematics, m_gyro.getYaw(), getModulePositions(), new Pose2d(), );

    public SwerveSubsystem(){
        m_modules = new SwerveModule[]{
            flModule, frModule, rlModule, rrModule
        };

        m_field = new Field2d();
        SmartDashboard.putData("Field", m_field);
    }

    public void drive(double vxMeters, double vyMeters, double omegaRadians, boolean fieldRelative, boolean isOpenLoop){
        ChassisSpeeds targetChassisSpeeds = fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(vxMeters, vyMeters, omegaRadians, getHeading())
            : new ChassisSpeeds(vxMeters, vyMeters, omegaRadians);
        setChassisSpeeds(targetChassisSpeeds, isOpenLoop, isOpenLoop);
    }

    public void setChassisSpeeds(ChassisSpeeds targetChassisSpeeds, boolean openLoop, boolean steerInPlace){
        setModuleStates(Constants.SwerveKinematics.kKinematics.toSwerveModuleStates(targetChassisSpeeds), openLoop, steerInPlace);
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
        }
    }

    public Rotation2d getHeading(){
        return Rotation2d.fromDegrees(getGyroYaw());
    }

    private double getGyroYaw(){
        return m_gyro.getYaw();
    }

    public Command teleopDrive(
            DoubleSupplier translation, DoubleSupplier rotation, DoubleSupplier strafe,
            BooleanSupplier robotCentric, BooleanSupplier openLoop){
        return run(() -> {
            double translationVal = MathUtil.applyDeadband(translation.getAsDouble(), Constants.swerveDeadband);
            double rotationVal = MathUtil.applyDeadband(rotation.getAsDouble(), Constants.swerveDeadband);
            double strafeVal = MathUtil.applyDeadband(strafe.getAsDouble(), Constants.swerveDeadband);

            //finish up
        }).withName("Teleop Drive");
    }

    //Controller --> Chassispeeds --> Swerve Module State [x4]
    //Odometry
    //Field-based orientation
    //Drive command

    @Override
    public void periodic(){
        
    }
}


