package org.firstinspires.ftc.teamcode.subSystems;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.other.Globals.*;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.DoubleSupplier;

@Config
public class IntakeSubsystem extends SubsystemBase {

    private Servo intake, diffyLeft, diffyRight;

    private Telemetry telemetry;

    private double pitchAngle;
    private double rollAngle;

    public static int pitchAngleOffset = 0;
    public static int rollAngleOffset = 0;

    //the value in the parentheses is our desired angle range in degrees
//    private double diffyScalar = ((245.0/255.0 * 355.0/360.0)/360); //servoRange *
//    private double diffyScalar = (255/245.0) * (360.0/355.0)/360.0;
    private double diffyScalar = 1.0/355.0 * 1.2;

    //intake rotation
    private int intakePitchAngle = 0;

    public IntakeSubsystem(Servo intake, Servo diffyLeft, Servo diffyRight, Telemetry telemetry) {
        this.intake = intake;
        this.diffyLeft = diffyLeft;
        this.diffyRight = diffyRight;
        this.telemetry = telemetry;
    }

    public void openClaw (){
        intake.setPosition(clawOpen);
    }

    public void closeClaw (){
        intake.setPosition(clawClose);
    }

    public void halfCloseClaw() {
        intake.setPosition(clawHalfClose);
    }

    public void clawExtraOpen () {
        intake.setPosition(clawExtraOpen);
    }

    public void setDiffy(double pitchAngle, double rollAngle){
        rollAngle/= 52/18.0;

        this.pitchAngle = pitchAngle;
        this.rollAngle = rollAngle;

        //pitch is like the wrist and roll is like the twist             if that makes any sense at all

        powerDiffyServos();
    }

    public void setDiffy(double rollAngle){
        rollAngle/= 52/18.0;

        this.rollAngle = rollAngle;

        //pitch is like the wrist and roll is like the twist             if that makes any sense at all

        powerDiffyServos();
    }

    public void setPitch(double pitchAngle) {
        this.pitchAngle = pitchAngle;

        powerDiffyServos();
    }


    public void powerDiffyServos(){
        double leftInput = (  ((pitchAngle + pitchAngleOffset) - (rollAngle + rollAngleOffset))  * diffyScalar + .5);
        double rightInput = (  ((pitchAngle + pitchAngleOffset) + (rollAngle + rollAngleOffset))  * diffyScalar + .5);
        diffyLeft.setPosition(leftInput);
        diffyRight.setPosition(rightInput);
//        Log.i("diffyServosSet", "true");
//        Log.i("diffyServosScalar", String.valueOf(diffyScalar));
//        Log.i("diffyServosLeft", String.valueOf(leftInput));
//        Log.i("diffyServosRight", String.valueOf(rightInput));
//        Log.i("diffyServosPitch", String.valueOf(pitchAngle));
//        Log.i("diffyServosRoll", String.valueOf(rollAngle));


    }


    public void resetRotateIntake(){
        intakePitchAngle = 0;
    }

    public void rotateIntake(){
        //switching to next rotation
        intakePitchAngle += 45;

        //rotation reset
        if (intakePitchAngle > 135){
            intakePitchAngle = 0;
        }
        switch (intakePitchAngle){
            case 0:
                setDiffy(0);
                break;
            case 45:
                setDiffy(45);
                break;
            case 90:
                setDiffy(90);
                break;
            case 135:
                setDiffy(135);
                break;
        }
    }

    //if another method is controlling the diffy, this method shouldn't be controlling it
    public void rotateIntake(boolean fullControl){
        if(fullControl){
            rotateIntake();
        }
        else{
            //switching to next rotation
            intakePitchAngle += 45;

            //rotation reset
            if (intakePitchAngle > 135){
                intakePitchAngle = 0;
            }

            //don't setDiffy here, cause normalizeRollToSecondaryArm will
        }
    }

    //Takes the yaw of the secondary arm and normalizes the wrist roll
    public void normalizeRollToSecondaryArm(double yawSupplier){
        double wristAngle = -yawSupplier;
        Log.i("wristAnglePreRotation", String.valueOf(wristAngle));
        wristAngle += intakePitchAngle;

        if(wristAngle>=105){
            while(wristAngle>=105){
                wristAngle-=180;
            }
        }
        else if (wristAngle<=-105){
            while(wristAngle<=-105){
                wristAngle+=180;
            }
        }

        Log.i("wristAnglePostRotation", String.valueOf(wristAngle));


        setDiffy(wristAngle);
    }

    @Override
    public void periodic() {
        telemetry.addData("pitchAngle", pitchAngle);
        telemetry.addData("rollAngle", rollAngle);
        telemetry.addData("clawPos", intake.getPosition());

        Log.i("diffyLeftServoPos", String.valueOf(diffyLeft.getPosition()));
        Log.i("diffyRightServoPos", String.valueOf(diffyRight.getPosition()));
    }

}
