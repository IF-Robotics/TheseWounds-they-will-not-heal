package org.firstinspires.ftc.teamcode.subSystems;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.other.Globals.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class IntakeSubsystem extends SubsystemBase {

    private Servo intake, diffyLeft, diffyRight;

    private Telemetry telemetry;

    private double pitchAngle;
    private double rollAngle;

    public static int pitchAngleOffset = 0;
    public static int rollAngleOffset = 0;

    //the value in the parentheses is our desired angle range in degrees
    public static double diffyScalar = (240/255 * 355) ; //servoRange *

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
        this.pitchAngle = pitchAngle;
        this.rollAngle = rollAngle;

        //pitch is like the wrist and roll is like the twist             if that makes any sense at all

        //accounting for the fact that we are using 2:1 bevel gears
        pitchAngle/= 2;

        diffyLeft.setPosition((((pitchAngle + pitchAngleOffset) - (rollAngle + rollAngleOffset)) / 2) * diffyScalar);
        diffyRight.setPosition((((pitchAngle + pitchAngleOffset) + (rollAngle + rollAngleOffset)) / 2) * diffyScalar);
    }

    public void setDiffy(double pitchAngle){
        this.pitchAngle = pitchAngle;

        //pitch is like the wrist and roll is like the twist             if that makes any sense at all

        //accounting for the fact that we are using 2:1 bevel gears
        pitchAngle/= 52/18;

        diffyLeft.setPosition((((pitchAngle + pitchAngleOffset) - (rollAngle + rollAngleOffset)) / 2) * diffyScalar);
        diffyRight.setPosition((((pitchAngle + pitchAngleOffset) + (rollAngle + rollAngleOffset)) / 2) * diffyScalar);
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

    @Override
    public void periodic() {
        telemetry.addData("pitchAngle", pitchAngle);
        telemetry.addData("rollAngle", rollAngle);
        telemetry.addData("clawPos", intake.getPosition());
    }

}
