package frc.robot.ShamLib.SMF.states;

public class StandardState<E extends Enum<E>> extends StateBase{
    public StandardState(Enum value) {
        super(value);
    }

    @Override
    public String getType() {
        return "standard";
    }
}
