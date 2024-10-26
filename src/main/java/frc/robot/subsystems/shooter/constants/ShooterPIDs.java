package frc.robot.subsystems.shooter.constants;

import frc.lib.util.TunableNumber;

public class ShooterPIDs {
    public static final TunableNumber SHOOTER_SHOOTER_kP
            = new TunableNumber("Shooter Shooter kP");
    public static final TunableNumber SHOOTER_SHOOTER_ACCELERATION
            = new TunableNumber("Shooter Shooter Acceleration");
    public static final TunableNumber SHOOTER_SHOOTER_JERK
            = new TunableNumber("Shooter Shooter Jerk");
    public static final TunableNumber SHOOTER_PIVOT_kP
            = new TunableNumber("Shooter Pivot kP");
    public static final TunableNumber SHOOTER_PIVOT_kI
            = new TunableNumber("Shooter Pivot kI");
    public static final TunableNumber SHOOTER_PIVOT_kD
            = new TunableNumber("Shooter Pivot kD");
    public static final TunableNumber SHOOTER_PIVOT_VELOCITY
            = new TunableNumber("Shooter Pivot Velocity");
    public static final TunableNumber SHOOTER_PIVOT_ACCELERATION
            = new TunableNumber("Shooter Pivot Acceleration");
}
