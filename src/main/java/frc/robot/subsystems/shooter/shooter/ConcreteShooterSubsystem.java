package frc.robot.subsystems.shooter.shooter;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.subsystems.shooter.constants.ShooterConstants;
import frc.robot.subsystems.shooter.constants.ShooterPIDs;
import frc.robot.subsystems.shooter.constants.ShootingMode;
import frc.robot.util.Helpers;

public class ConcreteShooterSubsystem extends ShooterSubsystem {
    private final TalonFX shooterLeft, shooterRight;
    private final TalonFXConfiguration config;
    private ShootingMode mode;

    public ConcreteShooterSubsystem() {
        shooterLeft = new TalonFX(ShooterConstants.IDs.SHOOTER_SHOOTER_LEFT_MOTOR, "Canivore1");
        shooterRight = new TalonFX(ShooterConstants.IDs.SHOOTER_SHOOTER_RIGHT_MOTOR, "Canivore1");
        shooterLeft.setInverted(true);
        var leftConfigurator = shooterLeft.getConfigurator();
        var rightConfigurator = shooterRight.getConfigurator();
        config = new TalonFXConfiguration()
                .withSlot0(
                        new Slot0Configs()
                                .withKP(
                                        ShooterPIDs.SHOOTER_SHOOTER_kP.get()
                                )
                )
                .withMotionMagic(
                        new MotionMagicConfigs()
                                .withMotionMagicAcceleration(
                                        ShooterPIDs.SHOOTER_SHOOTER_ACCELERATION.get()
                                )
                                .withMotionMagicJerk(
                                        ShooterPIDs.SHOOTER_SHOOTER_JERK.get()
                                )

                );
        leftConfigurator.apply(config);
        rightConfigurator.apply(config);
        shooterRight.setControl(new Follower(shooterLeft.getDeviceID(), true));
        mode = ShootingMode.IDLE;
    }

    private void setSpeed(double speed) {
        shooterLeft.setControl(new MotionMagicVelocityVoltage(speed));
    }

    @Override
    protected void setShootingMode(ShootingMode mode) {
        this.mode = mode;
    }

    @Override
    protected boolean atSpeedSetpoint() {
        return Helpers.withinTolerance(
                shooterLeft.getVelocity().getValueAsDouble(),
                getSpeed(mode),
                ShooterConstants.SHOOTER_SPEED_ERROR
        );
    }

    @Override
    public void periodic() {
        setSpeed(getSpeed(mode));
        updateTunables();
    }

    @Override
    public double getCurrentSpeed() {
        return shooterLeft.getVelocity().getValueAsDouble();
    }

    private void updateTunables() {
        config
                .withSlot0(
                        new Slot0Configs()
                                .withKP(
                                        ShooterPIDs.SHOOTER_SHOOTER_kP.get()
                                )
                )
                .withMotionMagic(
                        new MotionMagicConfigs()
                                .withMotionMagicAcceleration(
                                        ShooterPIDs.SHOOTER_SHOOTER_ACCELERATION.get()
                                )
                                .withMotionMagicJerk(
                                        ShooterPIDs.SHOOTER_SHOOTER_JERK.get()
                                )
                );
        shooterLeft.getConfigurator().apply(config);
        shooterRight.getConfigurator().apply(config);
    }
}
