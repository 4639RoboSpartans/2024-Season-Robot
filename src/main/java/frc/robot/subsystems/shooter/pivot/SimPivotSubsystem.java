package frc.robot.subsystems.shooter.pivot;

import frc.robot.subsystems.shooter.constants.ShootingMode;

public class SimPivotSubsystem extends PivotSubsystem {
    private ShootingMode mode;

    public SimPivotSubsystem() {
        mode = ShootingMode.IDLE;
    }

    @Override
    protected void setShootingMode(ShootingMode mode) {
        this.mode = mode;
    }

    @Override
    protected boolean atAngleSetpoint() {
        return true;
    }

    @Override
    public double getCurrentAngle() {
        return getAngle(mode);
    }
}
