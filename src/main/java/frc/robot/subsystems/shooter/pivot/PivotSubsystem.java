package frc.robot.subsystems.shooter.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.subsystems.shooter.constants.ShooterConstants;
import frc.robot.subsystems.shooter.constants.ShootingMode;
import frc.robot.util.AimUtil;

import static frc.robot.subsystems.shooter.constants.ShooterConstants.ShooterLowerOffset;

public abstract class PivotSubsystem extends SubsystemBase {
    private static PivotSubsystem instance;

    public static PivotSubsystem getInstance() {
        if (instance == null) {
            if (Robot.isReal()) instance = new ConcretePivotSubsystem();
            else instance = new SimPivotSubsystem();
        }
        return instance;
    }

    public Command setShootingModeCommand(ShootingMode mode) {
        return Commands.runOnce(() -> setShootingMode(mode));
    }

    public Trigger atAngleTrigger() {
        return new Trigger(this::atAngleSetpoint);
    }

    protected abstract void setShootingMode(ShootingMode mode);

    protected abstract boolean atAngleSetpoint();

    public abstract double getCurrentAngle();

    double getAngle(ShootingMode mode) {
        return switch (mode) {
            case IDLE -> ShooterLowerOffset;
            case AUTO, SPINUP -> AimUtil.getShooterSetpoint().angle();
            case MANUAL -> ShooterConstants.SHOOTER_MANUAL_ANGLE;
            case LAUNCH -> ShooterConstants.SHOOTER_LAUNCH_ANGLE;
        };
    }
}
