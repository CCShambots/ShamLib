package frc.robot.ShamLib;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.HashMap;
import java.util.Map;

public class AutonomousLoader<C extends CommandBase, E extends Enum<E>> {
    Map<E, C> autoRoutes = new HashMap<>();

    private final SendableChooser chooser;

    public AutonomousLoader(Map<E, C> autoRoutes) {
        //Routes
        this.autoRoutes = autoRoutes;


        this.chooser = composeSendableChooser();
    }

    private SendableChooser composeSendableChooser() {
        int iterations = 0;
        SendableChooser chooser = new SendableChooser();
        for(Map.Entry<E, C> e : autoRoutes.entrySet()) {
            if(iterations == 0) chooser.setDefaultOption(e.getKey().name(), e.getKey());
            else chooser.addOption(e.getKey().name(), e.getKey());
            iterations++;
        }

        return chooser;
    }

    public SendableChooser getSendableChooser() {
        return chooser;
    }

    public C getCurrentSelection() {
        return autoRoutes.get(chooser.getSelected());
    }


}
