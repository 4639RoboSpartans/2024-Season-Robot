package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Controls;
import frc.robot.constants.Controls.DriverControls;
import frc.robot.constants.RobotInfo.SwerveInfo;
import frc.robot.subsystems.swerve.ISwerveDriveSubsystem;
import frc.robot.util.AimUtil;

public class TeleopSwerveDriveCommand extends Command {
    private final ISwerveDriveSubsystem swerveDriveSubsystem;
    private final PIDController RotationPID;

    public TeleopSwerveDriveCommand(ISwerveDriveSubsystem swerveDriveSubsystem) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        RotationPID = SwerveInfo.TeleopRotationPID.create();
        addRequirements(swerveDriveSubsystem);
    }

    @Override
    public void initialize() {
        swerveDriveSubsystem.stop();
    }

    @Override
    public void execute() {
        double forwardsSpeed = DriverControls.SwerveForwardAxis.getAsDouble() * SwerveInfo.CURRENT_MAX_ROBOT_MPS;
        double sidewaysSpeed = DriverControls.SwerveStrafeAxis.getAsDouble() * SwerveInfo.CURRENT_MAX_ROBOT_MPS;

        if (DriverControls.SOTF.getAsBoolean()){
            forwardsSpeed /= 4;
            sidewaysSpeed /= 4;
        }

        double rotationMultiplier = Math.hypot(forwardsSpeed, sidewaysSpeed) / 2;
        double rotateSpeed = getRotationSpeed(sidewaysSpeed, rotationMultiplier);

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(forwardsSpeed, sidewaysSpeed, rotateSpeed);
        swerveDriveSubsystem.setFieldCentricMovement(chassisSpeeds);
        SmartDashboard.putBoolean("canShoot", Controls.canSOTF.getAsBoolean());
        SmartDashboard.putBoolean("aligned", Controls.aligned.getAsBoolean());
        SmartDashboard.putBoolean("in sector", Controls.inShootingSector.getAsBoolean());
        SmartDashboard.putBoolean("in range", Controls.inShootingRange.getAsBoolean());
    }

    private double getRotationSpeed(double sidewaysSpeed, double rotationMultiplier) {
        double rawSpeed;
        Rotation2d heading = swerveDriveSubsystem.getRotation2d();
        Rotation2d speaker = AimUtil.getSpeakerRotation();
        SmartDashboard.putNumber("speaker angle", speaker.getDegrees());
        SmartDashboard.putNumber("speaker x", AimUtil.getSpeakerVector().getX());
        SmartDashboard.putNumber("speaker y", AimUtil.getSpeakerVector().getY());
        SmartDashboard.putNumber("heading", heading.getDegrees());
        SmartDashboard.putNumber("speaker offset", heading.getDegrees() - speaker.getDegrees());
        if(DriverControls.SOTF.getAsBoolean()) {
            rawSpeed =  RotationPID.calculate(swerveDriveSubsystem.getRotation2d().getRadians(), AimUtil.getSpeakerRotation(sidewaysSpeed).getRadians());
        }
        else {
            rawSpeed = DriverControls.SwerveRotationAxis.getAsDouble() * SwerveInfo.TELOP_ROTATION_SPEED;
        }
        return rawSpeed * (1 + rotationMultiplier);
    }

    @Override
    public void end(boolean interrupted) {
        swerveDriveSubsystem.stop();
    }
}