package frc.robot.subsystems.shooter.pivot;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.subsystems.shooter.constants.ShooterConstants;
import frc.robot.subsystems.shooter.constants.ShooterPIDs;
import frc.robot.subsystems.shooter.constants.ShootingMode;
import frc.robot.util.Helpers;

public class ConcretePivotSubsystem extends PivotSubsystem {
    private final CANSparkMax pivotLeft, pivotRight;
    private final DutyCycleEncoder encoder;
    private final ProfiledPIDController pivotPID;

    private ShootingMode mode;

    public ConcretePivotSubsystem() {
        pivotLeft = new CANSparkMax(ShooterConstants.IDs.SHOOTER_PIVOT_MOTOR_LEFT,
                CANSparkMax.MotorType.kBrushless);
        pivotRight = new CANSparkMax(ShooterConstants.IDs.SHOOTER_PIVOT_MOTOR_RIGHT,
                CANSparkMax.MotorType.kBrushless);
        pivotLeft.setIdleMode(CANSparkBase.IdleMode.kBrake);
        pivotRight.setIdleMode(CANSparkBase.IdleMode.kBrake);
        pivotRight.follow(pivotLeft);

        encoder = new DutyCycleEncoder(ShooterConstants.IDs.SHOOTER_PIVOT_ENCODER_DIO_PORT);
        pivotPID = new ProfiledPIDController(
                ShooterPIDs.SHOOTER_PIVOT_kP.get(),
                ShooterPIDs.SHOOTER_PIVOT_kI.get(),
                ShooterPIDs.SHOOTER_PIVOT_kD.get(),
                new TrapezoidProfile.Constraints(
                        ShooterPIDs.SHOOTER_PIVOT_VELOCITY.get(),
                        ShooterPIDs.SHOOTER_PIVOT_ACCELERATION.get()
                )
        );
        mode = ShootingMode.IDLE;
    }

    private void setAngle(double angle) {
        pivotPID.setGoal(angle);
        double pidOutput = pivotPID.calculate(angle);
        pivotLeft.set(pidOutput);
    }

    @Override
    protected void setShootingMode(ShootingMode mode) {
        this.mode = mode;
    }

    @Override
    protected boolean atAngleSetpoint() {
        return Helpers.withinTolerance(
                encoder.get(),
                getAngle(mode),
                ShooterConstants.SHOOTER_PIVOT_ERROR
        );
    }

    @Override
    public double getCurrentAngle() {
        return encoder.get();
    }

    @Override
    public void periodic() {
        setAngle(getAngle(mode));
        updateTunables();
    }

    private void updateTunables() {
        pivotPID.setPID(
                ShooterPIDs.SHOOTER_PIVOT_kP.get(),
                ShooterPIDs.SHOOTER_PIVOT_kI.get(),
                ShooterPIDs.SHOOTER_PIVOT_kD.get()
        );
        pivotPID.setConstraints(
                new TrapezoidProfile.Constraints(
                        ShooterPIDs.SHOOTER_PIVOT_VELOCITY.get(),
                        ShooterPIDs.SHOOTER_PIVOT_ACCELERATION.get()
                )
        );
    }
}
