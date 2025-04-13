package org.firstinspires.ftc.teamcode.subSystems;


import android.util.Log;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.SecondaryArmCommand;

import java.util.function.DoubleSupplier;

@Config
public class SecondaryArmSubsystem extends SubsystemBase {

    private Servo secondaryArmLeft, secondaryArmRight, secondaryYawServo;

    private Telemetry telemetry;

    private double pitchAngle;
    private double yawAngle;

    public static int secondaryPitchAngleOffset = -54;
    public static int secondaryYawAngleOffset = 22;

    //the value in the parentheses is our desired angle range in degrees
//    public static double diffyScalar = (245/255 * 355)/360; //servoRange *
    public static double diffyScalar = 1.0/355.0 * 1.2;

    public static final double safeLowerPitch = 5.0;

    public static final int hardStoppedHighPitch = 200;

    public static final double secondaryArmLength = 5.2;

    public static final double extensionOffsetFromMiddle = -0.1;





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
        yawAngle = MathUtils.clamp(yawAngle, -90, 90);
//        this.yawAngle = yawAngle;
        this.yawAngle = 0;

        double yawInput = ((yawAngle + secondaryYawAngleOffset)*1.05*(95/90.0)/355 + .5);
        secondaryYawServo.setPosition(yawInput);
    }

    public void setDiffyYaw(DoubleSupplier yawAngle){
        double yawAngleDouble = MathUtils.clamp(yawAngle.getAsDouble(), -90, 90);
//        this.yawAngle = yawAngle;
        this.yawAngle = 0;

        double yawInput = ((yawAngleDouble + secondaryYawAngleOffset)/355 + .5);
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

    /**
     * If yaw is nonzero, checks and sets yaw to 0 before adjusting the pitch
     * @return conditional command beforming these checks
     */
    public Command setPitchSafe(double pitch){
        return new ConditionalCommand(
                new SecondaryArmCommand(this, pitch, 0),
                new SequentialCommandGroup(
                        new SecondaryArmCommand(this, this::getPitchAngle, 0),
                        new WaitCommand(300),
                        new SecondaryArmCommand(this, pitch, 0)
                ),
                () -> Math.abs(this.getYawAngle()) < 5
        );
    }

    /**
     * IF pitch is not within "yaw-safe" zones, will not even attempt to change the yaw
     */
    public Command setPitchYawSafe(double pitch, double yaw){
        //In this range, we cannot move yaw
        if (pitch > safeLowerPitch){
            return setPitchSafe(pitch);
        }
        else {
            //Just for slightly more efficiency + versatile
            if(Math.abs(yaw)>3){
                return setPitchSafe(pitch);
            }
            else {
                return new SecondaryArmCommand(this, pitch, yaw);
            }
        }
    }

    //returns yaw in RADIANS
    public double setX(double x){
        double yaw = Math.asin(x/secondaryArmLength);
        setDiffyYaw(Math.toDegrees(yaw));

        return yaw;
    }

    //returns how much more forward we move the slides for a certain yaw
    public double getSlideCompensation(double yaw){
        double circleDifference = 1 - Math.cos(yaw);

        return circleDifference*secondaryArmLength;
    }

}
