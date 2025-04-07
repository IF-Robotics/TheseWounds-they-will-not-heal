package org.firstinspires.ftc.teamcode.subSystems;

import static org.firstinspires.ftc.teamcode.other.Globals.*;
import static org.firstinspires.ftc.teamcode.other.Robot.voltageCompensation;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.LinkedList;
import java.util.function.DoubleSupplier;

@Config
public class ArmSubsystem extends SubsystemBase {

    private DcMotor slideL, slideR;
    private MotorGroup arm;
    private Servo endStop;
    private AnalogInput armEncoder;
    private Telemetry telemetry;


    //arm PIDF
    public static double kParm = 0.17, kIarm = 0, kDarm = 0.01, kFarm = 1, kGarm = 1.2; //kF is gain scheduling, kG is gravity ff
    public static double armWeakKP = 0.01;
    public static double armAngleOffset = -98;
    public static double climbingArmP = .03;
    private double armPowerCap = 1;
    private double ff;
    private PIDController armController;
    public static double setArmTargetAngle = 0;
    private double armPower;
    private double rawAngle;
    private double correctedAngle = 0;
    private InterpLUT slideKgLut = new InterpLUT();
    private InterpLUT slideKfLut = new InterpLUT();


    //slide pidf
    //IMPORTANT, slideKP needs to be changed in VisionToSampleInterpolte as well
    public static double slideKP = 1, slideKI = 0.0, slideKD = 0.0, slideKF = 0.07;
    private PIDController slideController;
    private final double ticksPerIn = 870.0/26.8; //one tick is about .04" which is about 1mm. This means that we have about 1mm of precision on the slides 738/30.5 / 1.3529
    private int slideTicks = 1;
    private double slidePower = 0;
    private double slideExtention = 9;
    public static double slideWristOffset = 9  ; //(in)
    public static final double slideRetractMin = slideWristOffset+0.5;
    public static double setSlideTarget = slideRetractMin;
    private double slideError = 0;

    //arm coordinates
    private double slideTargetIn;
    private double armTargetAngle;
    private double armHeight = 5.5;
    private double targetX = armFoldX;
    private double targetY = armFoldY;

    //manualArm
    private double armManualPower;
    private double slideManualPower;

    //intakeFromWall
    private boolean wallActive;

    //slide velocity
    private LinkedList<TimeStampedPosition> positionHistory = new LinkedList<>();
    private static final long VELOCITY_TIME_FRAME_MS = 100; // Time frame in milliseconds

    //nautilus
    private InterpLUT nautilus = new InterpLUT();   //nautilus theta to arm theta


    //last command store
    Command currentCommand;
    Command lastCommand;

    //constructor
    public ArmSubsystem(MotorGroup arm, DcMotor slideL, DcMotor slideR, Servo endStop, AnalogInput armEncoder, Telemetry telemetry) {
        this.arm = arm;
        this.slideL = slideL;
        this.slideR = slideR;
        this.endStop = endStop;
        this.armEncoder = armEncoder;
        this.telemetry = telemetry;
        wallActive = false;

//TODO: tune the slide gain scheduling
        //Adding each val with a key
        slideKgLut.add(-999999, 0);
        slideKgLut.add(9, 0);
        slideKgLut.add(20, 0.05);
        slideKgLut.add(25, .1);
        slideKgLut.add(30, .3);
        slideKgLut.add(35,.4);
        slideKgLut.add(40, .5);
        slideKgLut.add(9999999, .5);
        //generating final equation

        slideKgLut.createLUT();

        slideKfLut.add(-999999, 0.135);
        slideKfLut.add(7, 0.135);
        slideKfLut.add(23.9, .2);
        slideKfLut.add(41, .25);
        slideKfLut.add(99999999, .25);

        slideKfLut.createLUT();

        //nautilus lut
        nautilus.add(-999999,0);
        nautilus.add(0,0);
        nautilus.add(30,360);
        nautilus.add(999999,360);
        nautilus.createLUT();
    }

    public void manualArm(double armPower, double slidePower){
        armManualPower = -armPower;
        slideManualPower = slidePower;
    }

    public void manualSlides(double slidePower){
        slideManualPower = slidePower;
    }

    public void setArm(double targetAngle) {
        setArmTargetAngle = targetAngle;
    }

    public void setSlide(double targetInches) {
        if(targetInches<slideRetractMin){
            targetInches = slideRetractMin;
        }
        setSlideTarget = targetInches;
    }

    public void setArmCoordinates(double x, double y){
        targetX = x;
        targetY = y;

        //inverse kinematics
        slideTargetIn = Math.sqrt(Math.pow(x, 2) + Math.pow(y - armHeight, 2));
        armTargetAngle = Math.toDegrees(Math.atan2((y - armHeight), x));

        //write
        setArm(armTargetAngle);
        setSlide(slideTargetIn);
    }

    public void setArmY(double y){
        targetY = y;
        setArmCoordinates(targetX, y);
    }

    public void setArmX(double x){
        targetX = x;
        setArmCoordinates(x, targetY);
    }

    public void setArmX(DoubleSupplier x){
        targetX = x.getAsDouble();
        setArmCoordinates(targetX, targetY);
    }

    //set nautilus
    public void setNautilus(double theta){
        setArm(nautilus.get(theta));
    }

    public void setArmP(double p){
        armController.setP(p);
    }

    public void setArmPowerCap(double cap){
        armPowerCap = cap;
    }

    //forward kinematics
    public double getCurrentX(){
        return slideExtention * Math.cos(Math.toRadians(correctedAngle));
    }
    public double getCurrentY(){
        return slideExtention * Math.sin(Math.toRadians(correctedAngle)) + armHeight;
    }

    public double getArmAngle(){
        return correctedAngle;
    }

    public double getSlideExtention(){
        return slideExtention;
    }

    public double getSlideError(){
        return slideError;
    }

    public void setSlideP(double p){
        slideKP = p;
    }

    public double getSlideX(){
        return slideExtention * Math.cos(Math.toRadians(correctedAngle));
    }

    public double getArmTarget(){
        return setArmTargetAngle;
    }

    public double getSlideTarget(){
        return setSlideTarget;
    }

    public double getTargetX(){
        return targetX;
    }

    public double getTargetY(){
        return targetY;
    }

    //return intakeWall state
    public boolean getWallState(){
        return wallActive;
    }
    //toggle wall state
    public void toggleWallState(){
        wallActive = !wallActive;
    }


    //slide velocity
    public double getSlideVelocity() {

        long currentTime = System.currentTimeMillis();

        // Add the new position with its timestamp
        positionHistory.add(new TimeStampedPosition(getSlideExtention(), currentTime));

        // Remove old entries beyond the time frame
        while (!positionHistory.isEmpty() &&
                currentTime - positionHistory.getFirst().getTimestamp() > VELOCITY_TIME_FRAME_MS) {
            positionHistory.removeFirst();
        }


        if (positionHistory.size() < 2) {
            // Not enough data to calculate velocity
            return 0.0;
        }

        // Get the oldest and newest positions in the time frame
        TimeStampedPosition oldestPosition = positionHistory.getFirst();
        TimeStampedPosition newestPosition = positionHistory.getLast();

        // Calculate velocity: Δposition / Δtime
        double deltaPosition = newestPosition.getPosition() - oldestPosition.getPosition();
        double deltaTime = (newestPosition.getTimestamp() - oldestPosition.getTimestamp()) / 1000.0; // Convert ms to seconds

        // Avoid divide-by-zero errors
        if (deltaTime == 0) {
            return 0.0;
        }

        return deltaPosition / deltaTime; // Velocity in INCHES per second
    }

    public class TimeStampedPosition {
        private final double position; // For the arm, could be degrees or extension length
        private final long timestamp;  // Timestamp in milliseconds

        public TimeStampedPosition(double position, long timestamp) {
            this.position = position;
            this.timestamp = timestamp;
        }

        public double getPosition() {
            return position;
        }

        public long getTimestamp() {
            return timestamp;
        }

        @Override
        public String toString() {
            return "TimeStampedPosition{" +
                    "position=" + position +
                    ", timestamp=" + timestamp +
                    '}';
        }
    }

    public Command getLastCommand(){
        //redundant null checking
        if(lastCommand==null){
            return new InstantCommand();
        }
        return lastCommand;
    }

    @Override
    public void periodic() {
        //read
        slideTicks = slideR.getCurrentPosition();
        rawAngle = 360 - (armEncoder.getVoltage()/3.3 * 360);
        correctedAngle = rawAngle + armAngleOffset;

        //calculate slide extension
        slideExtention = (slideTicks/ticksPerIn + slideWristOffset) + 0.931 * Math.toRadians(correctedAngle);


        //arm pid
        armController = new PIDController(kParm * (kFarm * slideKfLut.get(slideExtention)), kIarm, kDarm);
        //feed forward
        ff = kGarm * (Math.cos(Math.toRadians(correctedAngle)) * slideKgLut.get(slideExtention));
        armPower = (voltageCompensation * (Math.sqrt(Math.abs(armController.calculate(correctedAngle, setArmTargetAngle))) * Math.signum(setArmTargetAngle - correctedAngle))) + ff;
        /*telemetry.addData("ff", ff);
        telemetry.addData("cos", Math.cos(Math.toRadians(angle)));;
        telemetry.addData("targetAngle", targetAngle);
        telemetry.addData("error", targetAngle - angle);*/
        slideError = setSlideTarget - slideExtention;
        telemetry.addData("slideError", slideError);

        //slide pid
        slideController = new PIDController(slideKP, slideKI, slideKD);
        slidePower = (voltageCompensation * slideController.calculate(slideExtention, setSlideTarget)) + (Math.sin(Math.toRadians(correctedAngle)) * slideKF);
        //telemetry.addData("targetIN", targetInches);
        //telemetry.addData("slideTicks", slideTicks);

        //arm manual
        if(manualArm){
            arm.set(armManualPower);
        } else {
            //pid power
            if(Math.abs(armPower) > armPowerCap){
                armPower = armPower * armPowerCap;
                arm.set(armPower);
            } else {
                arm.set(armPower);
            }
        }

        if(manualSlides){
            slideL.setPower(slideManualPower);
            slideR.setPower(slideManualPower);
        }
        else{
            slideL.setPower(slidePower);
            slideR.setPower(slidePower);
        }

        telemetry.addData("armAngle", correctedAngle);
        telemetry.addData("armTarget", setArmTargetAngle);
        telemetry.addData("armPower", armPower);
        telemetry.addData("armManual", armManualPower);
        telemetry.addData("armKP", armController.getP());
        telemetry.addData("armError", setArmTargetAngle - correctedAngle);

        telemetry.addData("slidePower", slidePower);
        telemetry.addData("slideExtention", slideExtention);

        telemetry.addData("targetArmX", targetX);
        telemetry.addData("targetArmY", targetY);
        telemetry.addData("xArmPos", getCurrentX());
        telemetry.addData("yArmPos", getCurrentY());
        telemetry.addData("slideVelocity", getSlideVelocity());




        //last command
        currentCommand = CommandScheduler.getInstance().requiring(this);

        if (currentCommand != null && currentCommand != lastCommand) {
            lastCommand = currentCommand;
        }
        //Redundent null checking
        if(lastCommand==null){
            lastCommand=new InstantCommand();
        }
        telemetry.addData("armSubsystemLastCommand", lastCommand != null ? lastCommand.getName() : "None");

        Log.i("slideLPower", String.valueOf(slideL.getPower()));
        Log.i("slideRPower", String.valueOf(slideR.getPower()));


    }

    public void setPowerZero(){
        arm.set(0);
    }

    public void setSlidePower(double power){
        slideL.setPower(power);
        slideR.setPower(power);
    }

    public void resetSlideEncoder(){
        slideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


}
