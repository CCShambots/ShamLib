package frc.robot.ShamLib;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.auto.FiveBall;
import frc.robot.commands.auto.OneBall;
import frc.robot.commands.auto.TwoBall;
import frc.robot.util.RobotManager;

import java.util.HashMap;
import java.util.Map;

public class AutonomousLoader<E extends Enum<E>> {
    Map<E, Command> autoRoutes = new HashMap<>();

    private SendableChooser chooser;

    public AutonomousLoader(Map<E, Command> autoRoutes) {
        //Routes
        this.autoRoutes = autoRoutes;


        this.chooser = composeSendableChooser();
    }

    private SendableChooser composeSendableChooser() {
        int iterations = 0;
        SendableChooser chooser = new SendableChooser();
        for(Map.Entry<E, Command> e : autoRoutes.entrySet()) {
            if(iterations == 0) chooser.setDefaultOption(e.getKey().name(), e.getKey());
            else chooser.addOption(e.getKey().name(), e.getKey());
            iterations++;
        }

        return chooser;
    }

    public SendableChooser getSendableChooser() {
        return chooser;
    }

    public Command getCurrentSelection() {
        return autoRoutes.get(chooser.getSelected());
    }


}
