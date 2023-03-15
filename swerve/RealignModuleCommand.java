package frc.robot.ShamLib.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.ArrayList;
import java.util.List;

import static frc.robot.ShamLib.ShamLibConstants.Swerve.ALLOWED_STOPPED_MODULE_DIFF;

public class RealignModuleCommand extends CommandBase {

    List<Double> positionList = new ArrayList<>();
    double minValue = 0;

    Timer timer = new Timer();
    boolean finished = false;

    SwerveModule module;

    public RealignModuleCommand(SwerveModule module) {
        this.module = module;
    }

    @Override
    public void initialize() {
        System.out.println("Irregularity detected on module " + module.getModuleName());
        positionList.clear();
        minValue = module.getAbsoluteAngle().getDegrees();
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        double thisValue = module.getAbsoluteAngle().getDegrees();

        positionList.add(thisValue);

        if(Math.abs(thisValue - minValue) > ALLOWED_STOPPED_MODULE_DIFF){
            System.out.println("FAILED TO CORRECT MODULE IRREGULARITY");
            finished = true;
        }

        if(timer.get() > 1) finished = true;

        if(thisValue < minValue) minValue = thisValue;
    }

    @Override
    public void end(boolean interrupted) {
        if(!interrupted) {
            double average = positionList.stream().mapToDouble((e) -> e).average().orElse(0);

            module.resetAngle(Rotation2d.fromDegrees(average));
        }

        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
