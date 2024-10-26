package frc.robot.subsystems.indexer.hopper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.subsystems.indexer.constants.IndexerState;

public abstract class HopperSubsystem extends SubsystemBase {
    private static HopperSubsystem instance;

    public static HopperSubsystem getInstance() {
        if (instance == null) {
            if (Robot.isReal()) instance = new ConcreteHopperSubsystem();
            else instance = new SimHopperSubsystem();
        }
        return instance;
    }

    public abstract void setIndexerState(IndexerState state);

    public abstract Trigger hasNoteTrigger();
}
