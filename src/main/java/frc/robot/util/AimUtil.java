package frc.robot.util;

import static frc.robot.constants.RobotInfo.ShooterInfo.ShooterSetpoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.shooter.ShooterMeasurementLERPer;

public class AimUtil {
  public static Rotation2d getSpeakerRotation() {
    Translation2d speakerVector = getSpeakerVector();
    return new Rotation2d(Math.tan(speakerVector.getY() / speakerVector.getX()));
  }

  public static Translation2d getSpeakerVector() {
    Pose2d currBotPose = SubsystemManager.getSwerveDrive().getPose();
    Translation2d currBotTranslation = currBotPose.getTranslation();
    Translation2d speakerPose;
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      speakerPose = FieldConstants.speakerPose_red;
    } else {
      speakerPose = FieldConstants.speakerPose_blue;
    }
    return speakerPose.minus(currBotTranslation);
  }

  public static Rotation2d getAmpRotation() {
    Translation2d speakerVector = getAmpVector();
    return new Rotation2d(Math.tan(speakerVector.getY() / speakerVector.getX()));
  }

  public static Translation2d getAmpVector() {
    Pose2d currBotPose = SubsystemManager.getSwerveDrive().getPose();
    Translation2d currBotTranslation = currBotPose.getTranslation();
    Translation2d speakerPose;
    if (DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      speakerPose = FieldConstants.ampPose_red;
    } else {
      speakerPose = FieldConstants.ampPose_blue;
    }
    return speakerPose.minus(currBotTranslation);
  }

  public static ShooterSetpoint getShooterSetpoint() {
    Translation2d speakerRelativeBotPose = getSpeakerVector();
    ShooterSetpoint result =
        ShooterMeasurementLERPer.get(speakerRelativeBotPose.getY(), speakerRelativeBotPose.getX());

    SmartDashboard.putNumber("Shooter Angle: ", result.angle());
    SmartDashboard.putNumber("Shooter Speed: ", result.speed());

    return result;
  }
}
