package frc.robot.ShamLib.swerve;

import edu.wpi.first.math.geometry.Translation2d;

public class ModuleInfo {

    final int driveMotorID;
    final int turnMotorID;
    final int encoderID;
    final double encoderOffset;
    final Translation2d offset;
    final double turnRatio;
    final double driveRatio;

    private double mk4iTurnRatio =
        (1.0 / 2048) * //Motor revs
        (7.0 / 150.0) * //Output revs
        360 //Output degrees
    ;

    private double mk4iWheelCircumference =
            2 * Math.PI * 0.0508;

    public ModuleInfo(int driveMotorID, int turnMotorID, int encoderID, double encoderOffset,
                      Translation2d offset, double turnRatio, double driveRatio) {
        this.driveMotorID = driveMotorID;
        this.turnMotorID = turnMotorID;
        this.encoderID = encoderID;
        this.encoderOffset = encoderOffset;
        this.offset = offset;
        this.turnRatio = turnRatio;
        this.driveRatio = driveRatio;
    }

    public ModuleInfo getL1Module(int driveMotorID, int turnMotorID, int encoderID, double encoderOffset, Translation2d offset) {
        return new ModuleInfo(driveMotorID, turnMotorID, encoderID, encoderOffset, offset,
                mk4iTurnRatio,
                (1.0 / 2048) * //Motor revs
                (1.0 / 8.14) * //Output revs
                mk4iWheelCircumference //Output meters
        );
    }

    public ModuleInfo getL2Module(int driveMotorID, int turnMotorID, int encoderID, double encoderOffset, Translation2d offset) {
        return new ModuleInfo(driveMotorID, turnMotorID, encoderID, encoderOffset, offset,
                mk4iTurnRatio,
                (1.0 / 2048) * //Motor revs
                (1.0 / 6.75) * //Output revs
                mk4iWheelCircumference //Output meters
        );
    }

    public ModuleInfo getL3Module(int driveMotorID, int turnMotorID, int encoderID, double encoderOffset, Translation2d offset) {
        return new ModuleInfo(driveMotorID, turnMotorID, encoderID, encoderOffset, offset,
                mk4iTurnRatio,
                (1.0 / 2048) * //Motor revs
                (1.0 / 6.12) * //Output revs
                mk4iWheelCircumference //Output meters
        );
    }
}
