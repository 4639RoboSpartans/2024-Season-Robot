package frc.robot.commands.shooter;

import frc.robot.constants.RobotInfo.ShooterInfo.ShootingMode;
import frc.robot.led.LEDStrip;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.shooter.IShooterSubsystem;

public class LaunchCommand extends ShootCommand{
    public LaunchCommand(IShooterSubsystem shooter, HopperSubsystem hopper, LEDStrip ledStrip){
        super(shooter, hopper, ledStrip, ShootingMode.LAUNCH, true);
    }
}
