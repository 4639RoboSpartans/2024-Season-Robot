package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotInfo;
import frc.robot.subsystems.shooter.IShooterSubsystem;

public class ShooterSpinupCommand extends Command {
  private final IShooterSubsystem shooterSubsystem;

  public ShooterSpinupCommand(IShooterSubsystem shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
    shooterSubsystem.setShootingMode(RobotInfo.ShooterInfo.ShootingMode.AUTO_SPEAKER);
  }
}
