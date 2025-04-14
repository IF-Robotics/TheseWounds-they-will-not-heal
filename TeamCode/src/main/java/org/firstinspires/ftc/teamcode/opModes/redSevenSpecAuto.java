package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subSystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.VisionSubsystem;

@Autonomous(name="Red 7+0❤️❤️❤️\uD83E\uDD16")
public class redSevenSpecAuto extends sevenSpecAuto {
    @Override
    public void initialize() {
        VisionSubsystem.alliance = VisionSubsystem.Alliance.RED;
        LimelightSubsystem.alliance = LimelightSubsystem.Alliance.RED;

        super.initialize();
    }
}
