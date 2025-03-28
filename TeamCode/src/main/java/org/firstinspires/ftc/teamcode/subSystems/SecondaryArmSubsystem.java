package org.firstinspires.ftc.teamcode.subSystems;


import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.SecondaryArmCommand;

@Config
public class SecondaryArmSubsystem extends SubsystemBase {

    private Servo secondaryArmLeft, secondaryArmRight;

    private Telemetry telemetry;

    private double pitchAngle;
    private double yawAngle;

    public static int secondaryPitchAngleOffset = 0;
    public static int secondaryYawAngleOffset = 0;

    //the value in the parentheses is our desired angle range in degrees
//    public static double diffyScalar = (245/255 * 355)/360; //servoRange *
    public static double diffyScalar = 1.0/355.0 * 1.2;



    public SecondaryArmSubsystem(Servo secondaryArmLeft, Servo secondaryArmRight, Telemetry telemetry) {
        this.secondaryArmLeft = secondaryArmLeft;
        this.secondaryArmRight = secondaryArmRight;
        this.telemetry = telemetry;
    }

    public void setDiffy(double pitchAngle, double yawAngle){

        this.pitchAngle = pitchAngle;
        setDiffyYaw(yawAngle);
    }

    public void setDiffyPitch(double pitchAngle){
        this.pitchAngle = pitchAngle;

        powerServos();
    }

    public void setDiffyYaw(double yawAngle){

        //accounting for the fact that we are using bevel gears with  GR
        yawAngle/= 52/18.0;

//        this.yawAngle = yawAngle;
        this.yawAngle = 0;
        powerServos();
    }

    public void powerServos(){
        double leftInput = (((pitchAngle + secondaryPitchAngleOffset) + (yawAngle + secondaryYawAngleOffset))  * diffyScalar + .5);
        double rightInput = (((pitchAngle + secondaryPitchAngleOffset) - (yawAngle + secondaryYawAngleOffset))  * diffyScalar + .5);
        secondaryArmLeft.setPosition(leftInput);
        secondaryArmRight.setPosition(rightInput);
//        secondaryArmLeft.setPosition((((pitchAngle + secondaryPitchAngleOffset) - (yawAngle + secondaryYawAngleOffset)) / 2) * diffyScalar +.5);
//        secondaryArmRight.setPosition((((pitchAngle + secondaryPitchAngleOffset) + (yawAngle + secondaryYawAngleOffset)) / 2) * diffyScalar + .5);
//        Log.i("secondaryArmServosSet", "true");
    }

    public void periodic() {
        telemetry.addData("secondaryArmPitchAngle", pitchAngle);
        telemetry.addData("secondaryArmRollAngle", yawAngle);

        Log.i("secondaryArmLeftServoPos", String.valueOf(secondaryArmLeft.getPosition()));
        Log.i("secondaryArmRightServoPos", String.valueOf(secondaryArmRight.getPosition()));
    }

    public Command intakeSub(){
        return new SecondaryArmCommand(this, 0, 0);
    }

}
