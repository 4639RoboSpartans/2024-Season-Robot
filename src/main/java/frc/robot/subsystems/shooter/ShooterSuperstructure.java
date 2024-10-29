package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.shooter.constants.ShooterConstants;
import frc.robot.subsystems.shooter.constants.ShootingMode;
import frc.robot.subsystems.shooter.pivot.PivotSubsystem;
import frc.robot.subsystems.shooter.shooter.ShooterSubsystem;

import java.util.Objects;

public class ShooterSuperstructure extends SubsystemBase {
    private static ShooterSuperstructure instance;

    public static ShooterSuperstructure getInstance() {
        return instance = Objects.requireNonNullElseGet(instance, ShooterSuperstructure::new);
    }

    private final ShooterSubsystem shooter;
    private final PivotSubsystem pivot;

    private final Mechanism2d shooterMech;
    private final MechanismLigament2d shooterArm;

    private ShooterSuperstructure() {
        shooter = ShooterSubsystem.getInstance();
        pivot = PivotSubsystem.getInstance();

        shooterMech = new Mechanism2d(6, 6);
        MechanismRoot2d shooterRoot = shooterMech.getRoot("Shooter Root", 1, 1);
        shooterArm = shooterRoot.append(
                new MechanismLigament2d(
                        "Shooter",
                        4.0,
                        getShooterRotation(pivot.getCurrentAngle()).getDegrees()
                )
        );
    }

    public Command setShootingMode(ShootingMode mode) {
        return shooter.setShootingModeCommand(mode)
                .alongWith(pivot.setShootingModeCommand(mode));
    }

    public Trigger atRequestedStateTrigger() {
        return shooter.atSpeedTrigger()
                .and(pivot.atAngleTrigger());
    }

    public static Rotation2d getShooterRotation(double shooterRotations) {
        double lowerOffset = ShooterConstants.ShooterLowerOffset;
        double higherOffset = ShooterConstants.ShooterLowerOffset - 0.095;
        return Rotation2d.fromRotations((shooterRotations - lowerOffset) / (higherOffset - lowerOffset) * 0.2 + 0.05);
    }

    @Override
    public void periodic() {
        shooterArm.setAngle(Rotation2d.fromRadians(pivot.getCurrentAngle()));
        SmartDashboard.putData("Shooter/Mech", shooterMech);
        SmartDashboard.putNumber("Shooter/Angle", getShooterRotation(pivot.getCurrentAngle()).getDegrees());
        SmartDashboard.putNumber("Shooter/Raw Angle", pivot.getCurrentAngle());
    }
}
