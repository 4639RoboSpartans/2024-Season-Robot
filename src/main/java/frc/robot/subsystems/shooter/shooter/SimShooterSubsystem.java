package frc.robot.subsystems.shooter.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.constants.ShooterConstants;
import frc.robot.subsystems.shooter.constants.ShooterPIDs;
import frc.robot.subsystems.shooter.constants.ShootingMode;

public class SimShooterSubsystem extends ShooterSubsystem {
    private ShootingMode mode;
    private final FlywheelSim flywheelSim;
    private final ProfiledPIDController flywheelPIDController;

    public SimShooterSubsystem() {
        mode = ShootingMode.IDLE;
        flywheelSim = new FlywheelSim(
                LinearSystemId.identifyVelocitySystem(0.0017932, 0.0001929),
                DCMotor.getFalcon500(2),
                1
        );
        flywheelPIDController = new ProfiledPIDController(
                ShooterPIDs.SHOOTER_SHOOTER_kP.get(),
                0, 0,
                new TrapezoidProfile.Constraints(
                        ShooterPIDs.SHOOTER_SHOOTER_ACCELERATION.get(),
                        ShooterPIDs.SHOOTER_SHOOTER_JERK.get()
                )
        );
    }

    @Override
    protected void setShootingMode(ShootingMode mode) {
        this.mode = mode;
    }

    @Override
    public Command setShootingModeCommand(ShootingMode mode) {
        return runOnce(() -> setShootingMode(mode));
    }

    @Override
    protected boolean atSpeedSetpoint() {
        return MathUtil.isNear(getCurrentSpeed(), getSpeed(mode), ShooterConstants.SHOOTER_SPEED_ERROR);
    }

    @Override
    public double getCurrentSpeed() {
        return flywheelSim.getAngularVelocityRadPerSec();
    }

    @Override
    public void periodic() {
        reachSetpoint();
        flywheelSim.update(0.020);
        flywheelSim.setState(flywheelSim.getAngularVelocityRPM());
    }

    private void reachSetpoint() {
        var pidOutput =
                flywheelPIDController.calculate(
                        flywheelSim.getAngularVelocityRadPerSec(),
                        getSpeed(mode)
                );
        flywheelSim.setInputVoltage(pidOutput);
    }

    @Override
    public double getTargetSpeed() {
        return getSpeed(mode);
    }
}
