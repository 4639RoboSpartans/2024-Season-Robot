package frc.robot.subsystems.shooter.constants;

public class ShooterConstants {
    public static double ShooterLowerOffset = 0.92;

    public static final double SHOOTER_SPINUP_SPEED = 0.2;

    public static final double SHOOTER_MANUAL_SPEED = 24.25,
            SHOOTER_MANUAL_ANGLE = ShooterLowerOffset - 0.095;

    public static final double SHOOTER_IDLE_SETPOINT = .85;

    public static final double SHOOTER_LAUNCH_SPEED = 30,
            SHOOTER_LAUNCH_ANGLE = 0.875;

    public static final double SHOOTER_PIVOT_ERROR = 0.01;
    public static final double SHOOTER_SPEED_ERROR = 0.025;

    public class IDs {
        public static final int SHOOTER_PIVOT_MOTOR_LEFT = 14;
        public static final int SHOOTER_PIVOT_MOTOR_RIGHT = 19;

        public static final int SHOOTER_SHOOTER_LEFT_MOTOR = 13;
        public static final int SHOOTER_SHOOTER_RIGHT_MOTOR = 17;

        public static final int SHOOTER_PIVOT_ENCODER_DIO_PORT = 0;
    }
}
