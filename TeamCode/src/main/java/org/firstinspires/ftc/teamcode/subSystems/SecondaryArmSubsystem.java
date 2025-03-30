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

    private Servo secondaryArmLeft, secondaryArmRight, secondaryYawServo;

    private Telemetry telemetry;

    private double pitchAngle;
    private double yawAngle;

    public static int secondaryPitchAngleOffset = -13;
    public static int secondaryYawAngleOffset = -73;

    //the value in the parentheses is our desired angle range in degrees
//    public static double diffyScalar = (245/255 * 355)/360; //servoRange *
    public static double diffyScalar = 1.0/355.0 * 1.2;



    public SecondaryArmSubsystem(Servo secondaryArmLeft, Servo secondaryArmRight, Telemetry telemetry, Servo secondaryArmServo) {
        this.secondaryArmLeft = secondaryArmLeft;
        this.secondaryArmRight = secondaryArmRight;
        this.secondaryYawServo = secondaryArmServo;
        this.telemetry = telemetry;
    }

    public void setDiffy(double pitchAngle, double yawAngle){

        this.pitchAngle = pitchAngle;
        setDiffyYaw(yawAngle);
        setDiffyPitch(pitchAngle);
    }

    public void setDiffyPitch(double pitchAngle){
        this.pitchAngle = pitchAngle;
        powerPitchServos();
    }

    public void setDiffyYaw(double yawAngle){

//        this.yawAngle = yawAngle;
        this.yawAngle = 0;

        double yawInput = ((yawAngle + secondaryYawAngleOffset)/355 + .5);
        secondaryYawServo.setPosition(yawInput);
    }

    public void powerPitchServos(){
        double leftInput = ((pitchAngle + secondaryPitchAngleOffset)  * diffyScalar + .5);
        double rightInput = ((pitchAngle + secondaryPitchAngleOffset)  * diffyScalar + .5);
        secondaryArmLeft.setPosition(leftInput);
        secondaryArmRight.setPosition(rightInput);
//        secondaryArmLeft.setPosition((((pitchAngle + secondaryPitchAngleOffset) - (yawAngle + secondaryYawAngleOffset)) / 2) * diffyScalar +.5);
//        secondaryArmRight.setPosition((((pitchAngle + secondaryPitchAngleOffset) + (yawAngle + secondaryYawAngleOffset)) / 2) * diffyScalar + .5);
//        Log.i("secondaryArmServosSet", "true");
    }

    public double getPitchAngle(){
        //return (secondaryArmLeft.getPosition() - .5)/diffyScalar - secondaryPitchAngleOffset;
        return pitchAngle;
    }

    public double getYawAngle(){
        //return (secondaryYawServo.getPosition() - .5) * 355 - secondaryYawAngleOffset;
        return yawAngle;
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
