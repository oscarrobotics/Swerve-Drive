package frc.robot.Swerve;

public enum SwerveConfig{

    //TODO: discover what the wheel reduction actually is
    SwerveX(
        0.1524,
        1,
        true,
        1,
        true
    );

    private final double wheelDiameter;
    private final double driveReduction;
    private final boolean driveInverted;
  
    private final double steerReduction;
    private final boolean steerInverted;
    
    /* 
    * 
    * @param wheelDiameter: the diameter of the wheel (meters)
    * 
    * @param driveReduction: the reduction of the drive motor
    * 
    * @param driveInverted: whether the drive motor is inverted or not
    * 
    * @param steerReduction: the reduction of the steer motor
    *  
    * @param steerInverted: whether the steer motor is inverted or not. If there are an odd number of reductions
    *                       then this should be true
     */

    private SwerveConfig(double wheelDiameter, double driveReduction, boolean driveInverted, 
        double steerReduction, boolean steerInverted) {
     this.wheelDiameter = wheelDiameter;
     this.driveReduction = driveReduction;
     this.driveInverted = driveInverted;
     this.steerReduction = steerReduction;
     this.steerInverted = steerInverted;
    }
    
    public double getWheelDiameter() {
        return wheelDiameter;
    }
    public double getDriveReduction() {
        return driveReduction;
    }
    public boolean getDriveInverted() {
        return driveInverted;
    }
    public double getSteerReduction() {
        return steerReduction;
    }
    public boolean getSteerInverted() {
        return steerInverted;
    }
}

