package org.firstinspires.ftc.teamcode.subSystems;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Optional;


@Config
public class LimelightSubsystem extends SubsystemBase {



    public enum Alliance {
        RED,
        BLUE

    }

    public static Alliance alliance = Alliance.RED;

    private final Limelight3A camera;
    private boolean isDataOld = false;
    private LLResult result;
    private double sampleColor = -1;


    public static double CAMERA_HEIGHT = 11.0-1.5;//limelight height minus height of sample (limelight detects top of sample)
    public static double CAMERA_ANGLE = 44.76; //downwards Angle

    Pose2d botToLimelight = new Pose2d(new Translation2d(6.5, 13.5/2.0-3.5), new Rotation2d());


//    public static double TARGET_HEIGHT = ;
//
//    public static double strafeConversionFactor = ;  //whatever numbers that the actual height and stuff is.
//    public static double cameraStrafeToBot = ;
//
//    public static double sampleToRobotDistance = ;

    Telemetry telemetry;


    public LimelightSubsystem(final HardwareMap hardwareMap, Telemetry telemetry) {
        camera = hardwareMap.get(Limelight3A.class, "limelight");

        this.telemetry = telemetry;

//        if (alliance == Alliance.BLUE){
//            camera.pipelineSwitch(0);  //pipeline 0 is blue
//            sampleColor = 0.0;
//        }
//        else{
//            camera.pipelineSwitch(1);  //pipeline 1 is red
//            sampleColor = 1.0;
//        }

        camera.pipelineSwitch(0);

        initializeCamera();
    } //pipeline 2 is yellow

    public void initializeCamera() {

        camera.setPollRateHz(50);
        camera.start();
    }

    @Override
    public void periodic() {
        if(camera.isRunning()){
            telemetry.addData("cameraNotRunning", "false");
            camera.updatePythonInputs(new double[] {sampleColor, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
            result = camera.getLatestResult(); // call this to get the limelight results
            if(result.isValid()) {
                Log.i("limelightValid", "true");
                long staleness = result.getStaleness();
                isDataOld = staleness >= 100; //100 ms
                telemetry.addData("TV", String.valueOf(getTv(result)));
                telemetry.addData("Tx", getTx(result));
                telemetry.addData("Ty", getTy(result));
                telemetry.addData("Angle", getAngle(result));
            }
        }
        else{
            telemetry.addData("cameraNotRunning", "true");
        }

        Optional<Pose2d> pose = getPose();
        if(pose.isPresent()) {
            telemetry.addData("right", pose.get().getX());
            telemetry.addData("forward", pose.get().getY());
            telemetry.addData("angle", pose.get().getRotation().getDegrees());
        }

        telemetry.update();
    }

    public Optional<LLResult> getResult(){
        if(camera.isRunning()) {
            return Optional.of(camera.getLatestResult());
        }
        return Optional.empty();
    }

    public double getTx(LLResult result) {
        return result.getTx();
    } //tx is the x distance from the crosshair (center of the screen)

    public double getTy(LLResult result) {
        return result.getTy();
    } //ty is y distance from the crosshair (center of the screen)

    //Whether see a sample or not
    public boolean getTv(LLResult result){
        if(result.getPythonOutput()[0]==1){
            return true;
        }
        return false;
    }

    public double getAngle(LLResult result) {
        double llAngle =  result.getPythonOutput()[3]; //0-180

        return llAngle;
    } //angle of the sample (.getPythonOutput()[i] gets the python snapscript outputs. 2 is center and 4 is area.

    public Optional<Pose2d> getPose(){

        Optional<LLResult> optionalResult = getResult();
        if(!optionalResult.isPresent()){return Optional.empty();}

        LLResult result = optionalResult.get();

        if(!getTv(result)){return Optional.empty();}

        //relative to limelight
        double forward = Math.tan(Math.toRadians(CAMERA_ANGLE+getTy(result)))*CAMERA_HEIGHT;

        double hypot = Math.hypot(CAMERA_HEIGHT, forward);

        double right = Math.tan(Math.toRadians(getTx(result)))*hypot;

        double angle = getAngle(result);

        angle += botToLimelight.getRotation().getDegrees();

        if(angle>90){
            while(angle>90){
                angle-=180;
            }
        }
        else if(angle<-90){
            while(angle<-90){
                angle+=180;
            }
        }

        //Makes it so counterclockwise is positive
        angle = -angle;

        Transform2d poseRelativeToLL = new Transform2d(new Translation2d(right, forward), new Rotation2d(Math.toRadians(angle)));

        Pose2d poseRelativeToBot = botToLimelight.transformBy(poseRelativeToLL);

        return Optional.of(poseRelativeToBot);
    }
}

