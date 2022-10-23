package frc.robot.ShamLib.swerve;

import edu.wpi.first.math.geometry.Translation2d;

public class ModuleInfo {

    final int driveMotorID;
    final int turnMotorID;
    final int encoderMotorID;
    final int encoderOffset;
    final Translation2d offset;

    public ModuleInfo(int driveMotorID, int turnMotorID, int encoderMotorID, int encoderOffset, Translation2d offset) {
        this.driveMotorID = driveMotorID;
        this.turnMotorID = turnMotorID;
        this.encoderMotorID = encoderMotorID;
        this.encoderOffset = encoderOffset;
        this.offset = offset;
    }
}
