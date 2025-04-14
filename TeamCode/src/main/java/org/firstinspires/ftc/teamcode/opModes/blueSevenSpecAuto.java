package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subSystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.VisionSubsystem;

@Autonomous(name="Blue 7+0\uD83D\uDC99\uD83E\uDD16\uD83D\uDC99\uD83E\uDD16\uD83D\uDC99\uD83E\uDD16")
public class blueSevenSpecAuto extends sevenSpecAuto{
    @Override
    public void initialize(){
        VisionSubsystem.alliance = VisionSubsystem.Alliance.BLUE;
        LimelightSubsystem.alliance = LimelightSubsystem.Alliance.BLUE;

        super.initialize();
    }
}
