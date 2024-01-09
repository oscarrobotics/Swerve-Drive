package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Swerve.SwerveSubsystem;

public class RobotContainer {
    private static RobotContainer m_robotContainer = new RobotContainer();
    private final CommandXboxController m_driverController = new CommandXboxController(0);
    
    public final SwerveSubsystem m_swerve = new SwerveSubsystem();

    public static RobotContainer getInstance() {
      return m_robotContainer;
    }
}
