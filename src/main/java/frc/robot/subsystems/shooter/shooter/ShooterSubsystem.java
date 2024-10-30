package frc.robot.subsystems.shooter.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.subsystems.shooter.constants.ShooterConstants;
import frc.robot.subsystems.shooter.constants.ShootingMode;
import frc.robot.util.AimUtil;

public abstract class ShooterSubsystem extends SubsystemBase {
    private static ShooterSubsystem instance;

    public static ShooterSubsystem getInstance() {
        if (instance == null) {
            if (Robot.isReal()) instance = new ConcreteShooterSubsystem();
            else instance = new SimShooterSubsystem();
        }
        return instance;
    }

    public abstract Command setShootingModeCommand(ShootingMode mode);

    public Trigger atSpeedTrigger() {
        return new Trigger(this::atSpeedSetpoint);
    }

    protected abstract void setShootingMode(ShootingMode mode);

    protected abstract boolean atSpeedSetpoint();

    public abstract double getCurrentSpeed();

    public static double getSpeed(ShootingMode mode) {
        return switch (mode) {
            case IDLE -> 0;
            case SPINUP -> ShooterConstants.SHOOTER_SPINUP_SPEED;
            case AUTO -> AimUtil.getShooterSetpoint().speed();
            case MANUAL -> ShooterConstants.SHOOTER_MANUAL_SPEED;
            case LAUNCH -> ShooterConstants.SHOOTER_LAUNCH_SPEED;
        };
    }

    public abstract double getTargetSpeed();
}
