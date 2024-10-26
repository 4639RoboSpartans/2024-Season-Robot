package frc.robot.subsystems.indexer.hopper;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.indexer.constants.IndexerConstants;
import frc.robot.subsystems.indexer.constants.IndexerState;

public class SimHopperSubsystem extends HopperSubsystem {
    private double speed;

    SimHopperSubsystem() {
        speed = 0;
    }

    @Override
    public void setIndexerState(IndexerState state) {
        speed = state.getHopperSpeed();
    }

    @Override
    public Trigger hasNoteTrigger() {
        return new Trigger(() -> true);
    }
}
