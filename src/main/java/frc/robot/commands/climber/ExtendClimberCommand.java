package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.climber.IClimberSubsystem;

public class ExtendClimberCommand extends Command {
    private final IClimberSubsystem climberSubsystem;

    public ExtendClimberCommand(IClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;

        addRequirements(climberSubsystem);
    }

    public void initialize() {
        climberSubsystem.stop();
    }

    public void execute() {
        climberSubsystem.setSpeed(1);
    }

    public void end(boolean interrupted) {
        climberSubsystem.stop();
    }
}