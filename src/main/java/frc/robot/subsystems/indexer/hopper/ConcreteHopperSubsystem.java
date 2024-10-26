package frc.robot.subsystems.indexer.hopper;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.indexer.constants.IndexerConstants;
import frc.robot.subsystems.indexer.constants.IndexerState;

public class ConcreteHopperSubsystem extends HopperSubsystem {
    private final CANSparkMax hopper;
    private final Trigger hasNoteTrigger;
    private IndexerState state;

    ConcreteHopperSubsystem() {
        hopper = new CANSparkMax(IndexerConstants.IDs.HOPPER_MOTOR, CANSparkMax.MotorType.kBrushed);
        DigitalInput ir = new DigitalInput(IndexerConstants.IDs.IR_SENSOR_1_DIO_PORT);
        hasNoteTrigger = new Trigger(ir::get).negate().debounce(0.5);
    }

    @Override
    public void setIndexerState(IndexerState state) {
        this.state = state;
        hopper.set(state.getHopperSpeed());
    }

    @Override
    public Trigger hasNoteTrigger() {
        return hasNoteTrigger;
    }
}
