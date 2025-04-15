package org.firstinspires.ftc.teamcode.subSystems;

import android.util.Log;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;
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
    private double sampleColor = -1;


    public static double CAMERA_HEIGHT = 11.23-1.5;//limelight height minus height of sample (limelight detects top of sample), 0.8 for offset cuz works?
    public static double CAMERA_ANGLE = 45.25; //downwards Angle

    Pose2d botToLimelight = new Pose2d(new Translation2d(6.556, 5.45), new Rotation2d(Math.toRadians(30)));


//    public static double TARGET_HEIGHT = ;
//
//    public static double strafeConversionFactor = ;  //whatever numbers that the actual height and stuff is.
//    public static double cameraStrafeToBot = ;
//
//    public static double sampleToRobotDistance = ;

    Telemetry telemetry;

    InterpLUT aspectLut = new InterpLUT();

    InterpLUT aspectLutX = new InterpLUT();

    InterpLUT tyCompensation = new InterpLUT();



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

        camera.pipelineSwitch(4);  //pipeline 1 is red

        initializeCamera();

        aspectLut.add(-999999, 2.5/2.0);
        aspectLut.add(-15, 2.5/2.0);
        aspectLut.add(-10, 3.0/2.5);
        aspectLut.add(-5, 2.9/2.0);
        aspectLut.add(0, 2.7/1.7);
        aspectLut.add(5, 2.3/1.7);
        aspectLut.add(10, 2.1/1.4);
        aspectLut.add(14, 1.8/1.2);
        aspectLut.add(99999999, 1.8/1.2);

        aspectLut.createLUT();

        aspectLutX.add(-999999, 1);
        aspectLutX.add(0, 1);
        aspectLutX.add(12, 1.5);
        aspectLutX.add(99999999, 1.5);

        aspectLutX.createLUT();

//        tyCompensation.add(-999999, -0.7);
//        tyCompensation.add(4.9, -0.7);
//        tyCompensation.add(5.5, -0.3);
//        tyCompensation.add(6.4, -0.25);
//        tyCompensation.add(7.4, -0.2);
//        tyCompensation.add(8.4, -0.15);
//        tyCompensation.add(9.3, -0.1);
//        tyCompensation.add(11.0, 0);
//        tyCompensation.add(9999999, 0);

        tyCompensation.add(-999999, -0.7);
        tyCompensation.add(4.9, -0.7);
        tyCompensation.add(5.5, -0.4);
        tyCompensation.add(6.4, -0.28);
        tyCompensation.add(7.4, -0.2);
        tyCompensation.add(8.4, -0.15);
        tyCompensation.add(9.3, -0.1);
        tyCompensation.add(11.0, 0);
        tyCompensation.add(9999999, 0);


        tyCompensation.createLUT();





    } //pipeline 2 is yellow

    public void initializeCamera() {

        camera.setPollRateHz(50);
        camera.start();
    }

    @Override
    public void periodic() {
//        if(camera.isRunning()){
//            telemetry.addData("cameraNotRunning", "false");
//            camera.updatePythonInputs(new double[] {sampleColor, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
//            Optional<LLResult> optionalResult = getResult(); // call this to get the limelight results
//            if(optionalResult.isPresent()) {
//                Log.i("limelightValid", "true");
//                LLResult result = optionalResult.get();
//                long staleness = result.getStaleness();
//                isDataOld = staleness >= 100; //100 ms
//            }
//        }
//        else{
//            telemetry.addData("cameraNotRunning", "true");
//        }
//
//        Optional<Pose2d> pose = getPose();
//        if(pose.isPresent()) {
//            telemetry.addData("right", pose.get().getX());
//            telemetry.addData("forward", pose.get().getY());
//            telemetry.addData("angle", pose.get().getRotation().getDegrees());
//        }

    }

    public Optional<LLResult> getResult(){
        if(camera.isRunning()) {
            LLResult result = camera.getLatestResult();
            if(result==null){
                return Optional.empty();
            }
            return Optional.of(result);
        }
        return Optional.empty();
    }

    public Optional<Pose2d> getPose(){

        Log.i("llbruh", "bruh");


        Optional<LLResult> optionalResult = getResult();
        if(!optionalResult.isPresent()){return Optional.empty();}

        LLResult results = optionalResult.get();

        ArrayList<Pose2d> poses = new ArrayList<Pose2d>();

        Log.i("llSize", String.valueOf(results.getDetectorResults().size()));

        for (LLResultTypes.DetectorResult result : results.getDetectorResults()) {

            Log.i("llresult", result.getClassName());
            if (alliance == Alliance.BLUE) {
                if (!result.getClassName().equals("blue")) {
                    continue;
                }
            } else if (alliance == Alliance.RED) {
                if (!result.getClassName().equals("red")) {
                    continue;
                }
            }

            //relative to limelight
            double forward = Math.tan(Math.toRadians(CAMERA_ANGLE + result.getTargetYDegrees())) * CAMERA_HEIGHT;
//            forward += tyCompensation.get(forward);

            telemetry.addData("forwardRaw", forward);

            double hypot = Math.hypot(CAMERA_HEIGHT, forward);

            double right = Math.tan(Math.toRadians(result.getTargetXDegrees())) * forward;

            telemetry.addData("rightRaw", right);

            List<List<Double>> corners = result.getTargetCorners();
            double angle = 0.0;

            if(!corners.isEmpty()){
                double changeInX = corners.get(0).get(0)-corners.get(2).get(0);
                double changeInY = corners.get(0).get(1)-corners.get(2).get(1);

                telemetry.addData("changeInX", changeInX);
                telemetry.addData("changeInY", changeInY);



                double aspectRatio = Math.abs(changeInY/changeInX);

                aspectRatio *= aspectLut.get(result.getTargetYDegrees());
                telemetry.addData("aspectRatio", aspectRatio);

                if(aspectRatio<1.0){
                    angle=90.0;
                }

                telemetry.addData("praywehittissampleangle", angle);
            }

//            telemetry.addData("anglePre", String.valueOf(angle));


            angle -= botToLimelight.getRotation().getDegrees(); //technically not field relative


//            Log.i("bruhAngle", String.valueOf(angle));

//        Transform2d poseRelativeToLL = new Transform2d(new Translation2d(right, forward), new Rotation2d(Math.toRadians(angle)));
//
//        Log.i("bruhAngleRelative", String.valueOf(poseRelativeToLL.getRotation().getDegrees()));
//        Pose2d poseRelativeToBot = botToLimelight.transformBy(poseRelativeToLL);
//        Log.i("bruhAngleAbsolute", String.valueOf(poseRelativeToBot.getRotation().getDegrees()));
//        telemetry.addData("bruhAngleAbsolute", poseRelativeToBot.getRotation().getDegrees());

            double cameraRadians = botToLimelight.getRotation().getRadians();

            double forwardBotRelative = Math.cos(cameraRadians) * forward + Math.sin(cameraRadians) * right + botToLimelight.getY();

            double rightBotRelative = -Math.sin(cameraRadians) * forward + Math.cos(cameraRadians) * right + botToLimelight.getX();


            Pose2d poseRelativeToBot = new Pose2d(rightBotRelative, forwardBotRelative, new Rotation2d(Math.toRadians(angle)));

            poses.add(poseRelativeToBot);
        }

        if(poses.size()<=0){
            return Optional.empty();
        }

        Pose2d best = new Pose2d();
        double lowestX = Double.MAX_VALUE;
        for (Pose2d pose : poses){
            double score = Math.abs(pose.getX()) + pose.getY()-ArmSubsystem.slideRetractMin;
            if(Math.abs(pose.getX())>SecondaryArmSubsystem.secondaryArmLength-0.5){
                score+=99999;
            }
            if (score<lowestX){
                lowestX = score;
                best = pose;
            }
        }

        return Optional.of(best);
    }

    public void pauseLimelight(boolean pause){
        if(pause){
            camera.pause();
        }
        else{
            camera.start();
        }
    }
}

