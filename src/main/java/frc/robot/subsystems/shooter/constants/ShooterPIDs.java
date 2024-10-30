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

    static {
        SHOOTER_PIVOT_kP.setDefault(34);
        SHOOTER_PIVOT_kI.setDefault(0.0);
        SHOOTER_PIVOT_kD.setDefault(1);
        SHOOTER_PIVOT_VELOCITY.setDefault(10);
        SHOOTER_PIVOT_ACCELERATION.setDefault(10);
        SHOOTER_SHOOTER_ACCELERATION.setDefault(3);
        SHOOTER_SHOOTER_JERK.setDefault(3);
        SHOOTER_PIVOT_kP.setDefault(0.0021693);
    }
}
