package org.firstinspires.ftc.teamcode.subSystems;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class SecondaryArmSubsystem extends SubsystemBase {

    private Servo secondaryArmLeft, secondaryArmRight;

    private Telemetry telemetry;

    private double pitchAngle;
    private double rollAngle;

    public static int pitchAngleOffset = 0;
    public static int rollAngleOffset = 0;

    //the value in the parentheses is our desired angle range in degrees
    public static double diffyScalar = (240/255 * 355) ; //servoRange *

    //intake rotation
    private int intakePitchAngle = 0;

    public SecondaryArmSubsystem(Servo secondaryArmLeft, Servo secondaryArmRight, Telemetry telemetry) {
        this.secondaryArmLeft = secondaryArmLeft;
        this.secondaryArmRight = secondaryArmRight;
        this.telemetry = telemetry;
    }

    public void setDiffy(double pitchAngle, double rollAngle){
        this.pitchAngle = pitchAngle;
        this.rollAngle = rollAngle;

        //accounting for the fact that we are using bevel gears with  GR
        pitchAngle/= 52/18;

        powerServos();
    }

    public void setDiffyPitch(double pitchAngle){
        this.pitchAngle = pitchAngle;

        //accounting for the fact that we are using bevel gears with  GR
        pitchAngle/= 52/18;

        powerServos();
    }

    public void setDiffyRoll(double rollAngle){
        this.rollAngle = rollAngle;

        //accounting for the fact that we are using bevel gears with  GR
        pitchAngle/= 52/18;

        powerServos();
    }

    public void powerServos(){
        secondaryArmLeft.setPosition((((pitchAngle + pitchAngleOffset) - (rollAngle + rollAngleOffset)) / 2) * diffyScalar);
        secondaryArmRight.setPosition((((pitchAngle + pitchAngleOffset) + (rollAngle + rollAngleOffset)) / 2) * diffyScalar);
    }




    @Override
    public void periodic() {
        telemetry.addData("secondaryArmPitchAngle", pitchAngle);
        telemetry.addData("secondaryArmRollAngle", rollAngle);
    }

}
