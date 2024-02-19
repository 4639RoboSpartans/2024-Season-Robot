package frc.robot.subsystems.shooterPivot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@SuppressWarnings("unused")
public class NeoShooterPivotSubsystem extends SubsystemBase implements IShooterPivotSubsystem {
    private final CANSparkMax aimMotor;
    private final DutyCycleEncoder encoder;
    private final PIDController aimPID;
    private boolean isUsingPID = true;

    public NeoShooterPivotSubsystem(int aimMotorID) {
        aimMotor = new CANSparkMax(aimMotorID, CANSparkMax.MotorType.kBrushless);
        aimMotor.setIdleMode(CANSparkBase.IdleMode.kCoast);
        encoder = new DutyCycleEncoder(0);
        aimPID = Constants.RobotInfo.SHOOTER_AIM_PID.create();
    }

    public void setAngleDegrees(double degrees) {
        aimPID.setSetpoint(degrees);
        isUsingPID = true;
    }

    public void manualSet(double speed){
        aimMotor.set(speed);
        isUsingPID = false;
    }

    @Override
    public void periodic() {
        if(!isUsingPID) return;

        double currentAimMotorDegrees = encoder.getAbsolutePosition();
        aimMotor.set(aimPID.calculate(currentAimMotorDegrees));

        SmartDashboard.putNumber("CurrentShooterAngle", currentAimMotorDegrees);
    }

    public void stop(){
        aimMotor.stopMotor();
    }
}