package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.climber.DummyClimberSubsystem;
import frc.robot.subsystems.climber.IClimberSubsystem;
import frc.robot.subsystems.hopper.DummyHopperSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.hopper.IHopperSubsystem;
import frc.robot.subsystems.intake.DummyIntakeSubsystem;
import frc.robot.subsystems.intake.IIntakeSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.sensors.IRSensor;
import frc.robot.subsystems.shooter.DummyShooterSubsystem;
import frc.robot.subsystems.shooter.FalconShooterSubsystem;
import frc.robot.subsystems.shooter.IShooterSubsystem;
import frc.robot.subsystems.shooter.pivot.DummyShooterPivotSubsystem;
import frc.robot.subsystems.shooter.pivot.IShooterPivotSubsystem;
import frc.robot.subsystems.shooter.pivot.NeoShooterPivotSubsystem;
import frc.robot.subsystems.swerve.AimSubsystem;
import frc.robot.subsystems.swerve.ISwerveDriveSubsystem;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

import static frc.robot.Constants.currentRobot;

public class SubsystemCreator {
    private static NavX navX;
    private static IRSensor irSensor;

    private static ISwerveDriveSubsystem swerveDrive;

    private static IShooterSubsystem shooter;
    private static IShooterPivotSubsystem shooterPivot;
    private static AimSubsystem aimSubsystem;
    private static IIntakeSubsystem intake;
    private static IHopperSubsystem hopper;
    private static IClimberSubsystem climber;

    public static NavX getNavX() {
        if(navX == null) {
            navX = new NavX();
        }
        return navX;
    }

    public static IRSensor getIRSensor() {
        if(irSensor == null) {
            irSensor = new IRSensor();
        }
        return irSensor;
    }

    public static ISwerveDriveSubsystem getSwerveDrive() {
        if(swerveDrive == null) {
            swerveDrive = new SwerveDriveSubsystem();
        }
        return swerveDrive;
    }

    public static IShooterPivotSubsystem getShooterPivot(IShooterSubsystem shooter) {
        if(shooterPivot == null) {
            shooterPivot = switch(Constants.currentRobot){
                case ZEUS -> new DummyShooterPivotSubsystem();
                case SIREN -> new NeoShooterPivotSubsystem(Constants.IDs.SHOOTER_PIVOT_MOTOR, shooter);
            };
        }
        return shooterPivot;
    }

    public static IShooterSubsystem getShooter() {
        if(shooter == null) {
            shooter = switch (currentRobot) {
                case ZEUS -> new DummyShooterSubsystem();
                case SIREN -> new FalconShooterSubsystem(
                    Constants.IDs.SHOOTER_SHOOTER_MOTOR
                );
            };
        }
        return shooter;
    }
    
    public static AimSubsystem getAimSubsystem() {
        if(aimSubsystem == null) {
            aimSubsystem = new AimSubsystem();
        }
        return aimSubsystem;
    }

    public static IIntakeSubsystem getIntake() {
        if(intake == null) {
            intake = switch(Constants.currentRobot){
                case ZEUS -> new DummyIntakeSubsystem();
                case SIREN -> new IntakeSubsystem(
                    Constants.IDs.INTAKE_PIVOT_MOTOR_LEFT,
                    Constants.IDs.INTAKE_PIVOT_MOTOR_RIGHT,
                    Constants.IDs.INTAKE_MOTOR,
                    Constants.IDs.INTAKE_ENCODER_CHANNEL
                );
            };
        }
        return intake;
    }

    public static IHopperSubsystem getHopper() {
        if(hopper == null) {
            hopper = switch(Constants.currentRobot){
                case ZEUS -> new DummyHopperSubsystem();
                case SIREN -> new HopperSubsystem(Constants.IDs.HOPPER_MOTOR);
            };
        }
        return hopper;
    }

    public static IClimberSubsystem getClimber() {
        if(climber == null) {
            climber = switch(Constants.currentRobot){
                case ZEUS -> new DummyClimberSubsystem();
                case SIREN -> new ClimberSubsystem(Constants.IDs.CLIMBER_LEFT, Constants.IDs.CLIMBER_RIGHT);
            };
        }
        return climber;
    }
}