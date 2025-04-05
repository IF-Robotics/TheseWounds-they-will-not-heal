package org.firstinspires.ftc.teamcode.subSystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Optional;

import android.graphics.Color;
import android.util.Log;
import android.util.Size;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.SortOrder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;


import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.teamcode.vision.CustomLocatorProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Core;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Config
public class VisionSubsystem extends SubsystemBase {

    public enum Alliance {
        RED,
        BLUE,
        NONE
    }

    public final static double kCameraWidth=1280;
    public final static double kDesiredX = kCameraWidth*0.5;

    public final static double kCameraHeight=720;
    public final static double kDesiredY = kCameraHeight*0.5;

    public static Alliance alliance = Alliance.BLUE;

    private final Servo light;
    Telemetry telemetry;


    public static int exposureMillis = 10;//8, 24, 35

    public static double lightStrength = 0.35;


    CustomLocatorProcessor customLocatorProcess;

    VisionPortal visionPortal;

    public VisionSubsystem(CameraName camera, Servo light, Telemetry telemetry){
        this.light = light;
        this.telemetry = telemetry;

        customLocatorProcess = new CustomLocatorProcessor();

        customLocatorProcess.setColor(CustomLocatorProcessor.COLOR.YELLOW);

        if(alliance == Alliance.BLUE) {
            customLocatorProcess.setColor(CustomLocatorProcessor.COLOR.BLUE);
        }
        else if (alliance ==Alliance.RED){
            customLocatorProcess.setColor(CustomLocatorProcessor.COLOR.RED);
        }

        visionPortal = new VisionPortal.Builder()
                .setCamera(camera)
                .setCameraResolution(new Size(1280, 720))
                .addProcessor(customLocatorProcess)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();
        enableLight(true);
        waitForSetCameraSettings(10000, 10000000);
    }


    @Override
    public void periodic(){
//        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
//            WhiteBalanceControl whiteBalanceControl = visionPortal.getCameraControl(WhiteBalanceControl.class);
//            telemetry.addData("Current White Balance Mode", whiteBalanceControl.getMode());
//            telemetry.addData("White Balance Temp", whiteBalanceControl.getWhiteBalanceTemperature());
//            telemetry.addData("Max temp", whiteBalanceControl.getMaxWhiteBalanceTemperature());
//            telemetry.addData("Min temp", whiteBalanceControl.getMinWhiteBalanceTemperature());
//            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
//            Log.i("gainCurr", String.valueOf(gainControl.getGain()));
//            Log.i("gainMax", String.valueOf(gainControl.getMaxGain()));
//            Log.i("gainMin", String.valueOf(gainControl.getMinGain()));
//            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
//            Log.i("exposureCurr", String.valueOf(exposureControl.getExposure(TimeUnit.MILLISECONDS)));
//            Log.i("exposureMax", String.valueOf(exposureControl.getMaxExposure(TimeUnit.MILLISECONDS)));
//            Log.i("exposureMin", String.valueOf(exposureControl.getMinExposure(TimeUnit.MILLISECONDS)));
//            Log.i("exposureAE", String.valueOf(exposureControl.getAePriority()));
//            Log.i("exposureMode", String.valueOf(exposureControl.getMode()));
//
//        }
//        telemetry.addData("Sample Skew", getTotalSkew().orElse(-99999.0));
//
        Optional<List<Double>> offsets = getOffsets();
        if(offsets.isPresent()) {
            telemetry.addData("offset x", offsets.get().get(0));
            telemetry.addData("offset y", offsets.get().get(1));
        }
//
        Optional<RotatedRect> rect = getBoxFit();
        if(rect.isPresent()){
            telemetry.addData("rect x", rect.get().center.x);
            telemetry.addData("rect y", rect.get().center.y);
        }
//        setExposure();
//        light.setPosition(lightStrength);
    }

    public Optional<List<Double>> getOffsets(){
        Optional<RotatedRect> desiredBoxFit = getBoxFit();
        if(!desiredBoxFit.isPresent()){
            return Optional.empty();
        }

        return Optional.of(getOffsetFromBoxFit(desiredBoxFit.get()));


    }

    public List<Double> getOffsetFromBoxFit(RotatedRect boxfit){

        double tx = boxfit.center.x-kDesiredX;
        double ty = boxfit.center.y-kDesiredY;

        return Arrays.asList(tx,ty);
    }

    public Optional<RotatedRect> getBoxFit(){
        List<RotatedRect> blobs = customLocatorProcess.getBlobs();
        if(blobs.isEmpty()){return Optional.empty();}
        return getClosestBoxFit(blobs);
    }

    public Optional<Double> getSkew(){
        Optional<RotatedRect> desiredBoxFit = getBoxFit();
        if (!desiredBoxFit.isPresent()){return Optional.empty();}

        return Optional.of(getAngleFromRotatedRect(desiredBoxFit.get()));
    }

    public double getAngleFromRotatedRect(RotatedRect boxFitBlob){
        //This math is essentially to find the angle considering that the long side is vertical would be 0 degrees
        //This math is likely unecessary but just to be safe added
        Point[] vertices = new Point[4];
        boxFitBlob.points(vertices);

//        for(int i = 0; i < 4; i++){
//            telemetry.addData("Vertex"+i+"X before change", vertices[i].x);
//            telemetry.addData("Vertex"+i+"Y before change", vertices[i].y);
//        }


        //Find the vertex with the max x value
        double max = Double.MIN_VALUE;
        int maxIndex = -1;
        for (int i = 0; i<4; i++){
            if(vertices[i].x > max){
                max = vertices[i].x;
                maxIndex = i;
            }
        }

        //Use this max x value to ensure that we're properly finding the two smallest x values
        int min1 = maxIndex;
        int min2 = maxIndex;
        for (int i=0; i<4; i++) {
            if (vertices[i].x < vertices[min1].x) {
                min2 = min1;
                min1 = i;
            } else if (vertices[i].x < vertices[min2].x) {
                min2 = i;
            }
        }

//        telemetry.addData("min1 index", min1);
//        telemetry.addData("min2 index", min2);

        //Ensure that indicies 0 and 1 are the minimum x values
        Point temp = vertices[0];
        vertices[0] = vertices[min1];
        vertices[min1] = temp;
        temp = vertices[1];
        vertices[1] = vertices[min2];
        vertices[min2] = temp;

//        for(int i = 0; i < 4; i++){
//            telemetry.addData("Vertex"+i+"X before change", vertices[i].x);
//            telemetry.addData("Vertex"+i+"Y before change", vertices[i].y);
//        }

        //Sometimes, if the x values are the same, it won't get organized properly. With this, we ensure that the other values will be organized properly
        //This is cause we want it to be organized as 0 and 1 are the lowest x values, but 3 is always farther to 0 than 2
        //The logic doesn't make sense because it was added as an afterthought
        double side2 = Math.hypot(
                (vertices[0].x - vertices[2].x),
                (vertices[0].y - vertices[2].y)
        );
        double side3 = Math.hypot(
                (vertices[0].x - vertices[3].x),
                (vertices[0].y - vertices[3].y)
        );
        //Ensure that the indices of 2 and 3 are organized by smaller x value
        if (side2 > side3) {
            temp = vertices[2];
            vertices[2] = vertices[3];
            vertices[3] = temp;
            side2 = side3;
        }

//        for(int i = 0; i < 4; i++){
//            telemetry.addData("Vertex"+i+"X after change", vertices[i].x);
//            telemetry.addData("Vertex"+i+"Y after change", vertices[i].y);
//        }
        //Find distances to find longer side of rectangle
        double side1 = Math.hypot(
                (vertices[0].x - vertices[1].x),
                (vertices[0].y - vertices[1].y)
        );

        double angle = 0;

        if(side1 > side2){
            angle = Math.atan((vertices[2].y - vertices[0].y)/(vertices[2].x - vertices[0].x));
        }
        else{
            angle = Math.atan((vertices[1].y - vertices[0].y)/(vertices[1].x - vertices[0].x));
        }

//        telemetry.addData("side1", side1);
//        telemetry.addData("side2", side2);
//        telemetry.addData("side1longer", side1>side2);
        return Math.toDegrees(angle);
    }

    public Optional<RotatedRect> getClosestBoxFit(List<RotatedRect> blobs) {
        if(blobs.isEmpty()){return Optional.empty();}

        double lowestDistance = Math.hypot(blobs.get(0).center.x-kDesiredX, blobs.get(0).center.y-kDesiredY);
        int lowestIndex = 0;
        for (int i=1; i<blobs.size(); i++){
            double distance = Math.hypot(blobs.get(i).center.x-kDesiredX, blobs.get(i).center.y-kDesiredY);
            if(distance<lowestDistance){
                lowestDistance=distance;
                lowestIndex=i;
            }
        }

        return Optional.of(blobs.get(lowestIndex));
    }

    public boolean setExposure() {
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            return false;
        }

        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
//        exposureControl.setMode(ExposureControl.Mode.Manual);
//        Log.i("camera", "exposure: " + exposureControl.getExposure(TimeUnit.MILLISECONDS));
        return exposureControl.setExposure(exposureMillis, TimeUnit.MILLISECONDS) && exposureControl.setMode(ExposureControl.Mode.Manual) && exposureControl.setAePriority(false);
    }

    public boolean setWhiteBalance() {
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            return false;
        }

        WhiteBalanceControl whiteBalanceControl = visionPortal.getCameraControl(WhiteBalanceControl.class);
        return whiteBalanceControl.setMode(WhiteBalanceControl.Mode.AUTO);
//        Log.i("camera", "white balance: " + whiteBalanceControl.getWhiteBalanceTemperature());
//        return whiteBalanceControl.setWhiteBalanceTemperature(8000);
    }

    public boolean waitForSetCameraSettings(long timeoutMs, int maxAttempts) {
        long startMs = System.currentTimeMillis();
        int attempts = 0;
        long msAfterStart = 0;
        boolean haveSetExposure = false;
        boolean haveSetWhiteBalance = false;
        while (msAfterStart < timeoutMs && attempts++ < maxAttempts) {
            if (!haveSetExposure && setExposure()) {
                haveSetExposure=true;
                Log.i("camera", "Exposure (individual) setting worked");

            }
            if(!haveSetWhiteBalance && setWhiteBalance()) {
                haveSetWhiteBalance=true;
            }
            if(haveSetExposure&&haveSetWhiteBalance){
                Log.i("camera", "Exposure setting worked");
                return true;
            }
            msAfterStart = System.currentTimeMillis() - startMs;
        }

        Log.e("camera", "Set exposure failed msAfterStart:" + String.valueOf(msAfterStart) + " attempts:" + String.valueOf(attempts));
        return false;
    }

    public void turnOnStreaming(boolean enabled){
        if(enabled){
            visionPortal.resumeStreaming();
        }
        else{
            visionPortal.stopStreaming();
        }
    }

    public void enableLight(boolean enabled){
        if(enabled){
            light.setPosition(0.15);
        }
        else{
            light.setPosition(0);
        }
    }
}