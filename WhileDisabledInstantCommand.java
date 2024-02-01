package frc.robot.ShamLib;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class WhileDisabledInstantCommand extends InstantCommand {

    public WhileDisabledInstantCommand(Runnable toRun) {
        super(toRun);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }


}
