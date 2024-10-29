package frc.robot.subsystems.shooter.pivot;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.constants.Constants;
import frc.robot.subsystems.shooter.ShooterSuperstructure;
import frc.robot.subsystems.shooter.constants.ShooterConstants;
import frc.robot.subsystems.shooter.constants.ShooterPIDs;
import frc.robot.subsystems.shooter.constants.ShootingMode;

public class SimPivotSubsystem extends PivotSubsystem {
    private ShootingMode mode;
    private final ProfiledPIDController pivotPID;
    private final SingleJointedArmSim armSim;

    public SimPivotSubsystem() {
        mode = ShootingMode.IDLE;
        armSim =new SingleJointedArmSim(
                LinearSystemId.createSingleJointedArmSystem(
                        DCMotor.getNeo550(2),
                        SingleJointedArmSim.estimateMOI(0.3, 10),
                        81.0 * 3 / 2
                ),
                DCMotor.getNEO(2),
                81.0 * 3 / 2,
                0.3,
                2 * Math.PI * 20 / 360,
                2 * Math.PI / 4,
                true,
                2 * Math.PI * 20 / 360
        );
        pivotPID = new ProfiledPIDController(
                ShooterPIDs.SHOOTER_PIVOT_kP.get(),
                ShooterPIDs.SHOOTER_PIVOT_kI.get(),
                ShooterPIDs.SHOOTER_PIVOT_kD.get(),
                new TrapezoidProfile.Constraints(
                        ShooterPIDs.SHOOTER_PIVOT_VELOCITY.get(),
                        ShooterPIDs.SHOOTER_PIVOT_ACCELERATION.get()
                )
        );
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
        return armSim.getAngleRads();
    }

    @Override
    public void periodic() {
        reachSetpoint();
        armSim.update(0.020);
        armSim.setState(armSim.getAngleRads(), armSim.getVelocityRadPerSec());
    }

    /** Run the control loop to reach and maintain the setpoint from the preferences. */
    public void reachSetpoint() {
        var pidOutput =
                pivotPID.calculate(
                        armSim.getAngleRads(),
                        ShooterSuperstructure.getShooterRotation(getAngle(mode)).getRadians()
                );
        armSim.setInputVoltage(pidOutput);
    }
}
