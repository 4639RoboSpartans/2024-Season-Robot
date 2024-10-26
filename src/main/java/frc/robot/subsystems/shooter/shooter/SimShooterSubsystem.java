package frc.robot.subsystems.shooter.shooter;

import frc.robot.subsystems.shooter.constants.ShootingMode;

public class SimShooterSubsystem extends ShooterSubsystem {
    private ShootingMode mode;

    public SimShooterSubsystem() {
        mode = ShootingMode.IDLE;
    }

    @Override
    protected void setShootingMode(ShootingMode mode) {
        this.mode = mode;
    }

    @Override
    protected boolean atSpeedSetpoint() {
        return true;
    }

    @Override
    public double getCurrentSpeed() {
        return getSpeed(mode);
    }
}
