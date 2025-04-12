package org.firstinspires.ftc.teamcode.opModes;

import static com.qualcomm.robotcore.hardware.Gamepad.LED_DURATION_CONTINUOUS;
import static org.firstinspires.ftc.teamcode.other.Globals.*;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.firstinspires.ftc.teamcode.commands.VisionClawCommand;
import org.firstinspires.ftc.teamcode.other.Robot;
import org.firstinspires.ftc.teamcode.opModes.TeleopOpMode;
import org.firstinspires.ftc.teamcode.subSystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.VisionSubsystem;

//import org.firstinspires.ftc.teamcode.subSystems.VisionSubsystem;
@TeleOp(name="BlueTeleop\uD83D\uDC99\uD83C\uDFAE\uD83D\uDC99\uD83C\uDFAE\uD83D\uDC99\uD83C\uDFAE")
public class blueTeleop extends TeleopOpMode {

    ElapsedTime setColorTimer = new ElapsedTime();
    public void initialize(){
        VisionSubsystem.alliance = VisionSubsystem.Alliance.BLUE;
        LimelightSubsystem.alliance = LimelightSubsystem.Alliance.BLUE;

        super.initialize();
    }

    @Override
    public void run(){
        super.run();

        if(teleopSpec){
            setColorTimer.reset();
            gamepad1.setLedColor(0,0,255, LED_DURATION_CONTINUOUS);
            Log.i("gamepadSetColorTime(mm)", String.valueOf(setColorTimer.milliseconds()));
        } else{
            setColorTimer.reset();
            gamepad1.setLedColor(255, 255, 0, LED_DURATION_CONTINUOUS);
            Log.i("gamepadSetColorTime(mm)", String.valueOf(setColorTimer.milliseconds()));
        }
    }


}
