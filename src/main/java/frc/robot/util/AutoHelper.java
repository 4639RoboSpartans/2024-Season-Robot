package frc.robot.util;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.shooter.AutoSpeakerCommand;
import frc.robot.commands.shooter.SOTFCommand;
import frc.robot.commands.shooter.ShooterSpinupCommand;
import frc.robot.constants.Controls;
import frc.robot.constants.RobotInfo;
import frc.robot.led.LEDStrip;
import frc.robot.oi.OI;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.hopper.IHopperSubsystem;
import frc.robot.subsystems.intake.IIntakeSubsystem;
import frc.robot.subsystems.shooter.IShooterSubsystem;
import frc.robot.subsystems.swerve.ISwerveDriveSubsystem;

public class AutoHelper {
//     private static ISwerveDriveSubsystem swerve = SubsystemManager.getSwerveDrive();
//     private static IShooterSubsystem shooter = SubsystemManager.getShooter();
//     private static IIntakeSubsystem intake = SubsystemManager.getIntake();
//     private static IHopperSubsystem hopper = SubsystemManager.getHopper();
//     private static LEDStrip ledStrip = SubsystemManager.getLedStrip();
//     private static OI oi = SubsystemManager.getOI();

//     public static Command follow(String pathName) {
//         return swerve.followChoreoPath(pathName, true);
//     }

//     public static Command intakeWhileMoving(String pathName) {
//         ChoreoTrajectory traj = Choreo.getTrajectory(pathName);
//         double time = traj.getTotalTime();
//         double startTime = time - RobotInfo.SwerveInfo.TIME_BEFORE_INTAKE_START;
//         Command ret = Commands.deadline(
//                 swerve.followChoreoPath(traj, true),
//                 startTime < 0 ?
//                         new IntakeCommand(intake, hopper, ledStrip, oi):
//                         Commands.sequence(
//                                 new WaitCommand(startTime),
//                                 new IntakeCommand(intake, hopper, ledStrip, oi)
//                         )
//         );
//         ret.addRequirements(
//                 SubsystemManager.getSwerveDrive(),
//                 SubsystemManager.getIntake(),
//                 SubsystemManager.getHopper());
//         return ret;
//     }

//     public static Command SOTFCommand(String pathName) {
//         /*
//          * max velocity of traj during SOTF portion must not be too high (<=2 m/s),
//          * should generate path so that bot is facing speaker during SOTF portion
//          */
//         Command shooterAutoSpinupCommand = new ShooterSpinupCommand(shooter).onlyWhile(Controls.spinupTrigger);
//         Command shooterSOTFCommand = new SOTFCommand(shooter, hopper, ledStrip);
//         Command ret =  Commands.deadline(
//                 intakeWhileMoving(pathName),
//                 shooterSOTFCommand
//         );
//         ret.addRequirements(
//                 SubsystemManager.getSwerveDrive(),
//                 SubsystemManager.getShooter(),
//                 SubsystemManager.getIntake(),
//                 SubsystemManager.getHopper());
//         return ret;
//     }

//     public static Command followThenShoot(String pathname) {
//         ChoreoTrajectory traj = Choreo.getTrajectory(pathname);
//         Command ret =  Commands.sequence(
//                 swerve.followChoreoPath(traj, true),
//                 new SOTFCommand(shooter, hopper, ledStrip)
//         );
//         ret.addRequirements(
//                 SubsystemManager.getSwerveDrive(),
//                 SubsystemManager.getShooter(),
//                 SubsystemManager.getHopper());
//         return ret;
//     }

//     public static Command shoot() {
//         return new AutoSpeakerCommand(shooter, hopper, ledStrip);
//     }
}
