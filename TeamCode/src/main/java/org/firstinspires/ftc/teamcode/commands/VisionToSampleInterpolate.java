package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.other.Globals.armReadySubIntakeY;
import static org.firstinspires.ftc.teamcode.other.Globals.armSubIntakeY;
import static org.firstinspires.ftc.teamcode.other.Globals.pitchWhenIntake;
import static org.firstinspires.ftc.teamcode.other.Globals.rollWhenIntake;

import android.provider.Settings;
import android.util.Log;

import androidx.core.math.MathUtils;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.canvas.Rotation;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.arcrobotics.ftclib.util.LUT;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.other.Globals;
import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.SecondaryArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.VisionSubsystem;
import org.opencv.core.RotatedRect;

import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

@Config
public class VisionToSampleInterpolate extends CommandBase {
    private DriveSubsystem driveSubsystem;
    private VisionSubsystem visionSubsystem;
    private ArmSubsystem armSubsystem;

    private IntakeSubsystem intakeSubsystem;

    private SecondaryArmSubsystem secondaryArmSubsystem;

    private BooleanSupplier slowMode;
    private DoubleSupplier strafe, forward, turn;


    public static double kPTurn = 0.005 * 180 / Math.PI;

    BasicPID turnpid = new BasicPID(new PIDCoefficients(kPTurn,0,0));

    private double prevSlidePosition = 0;

    private final double kPitchConversion = 2.33;

    private boolean offsetOnTarget = false;
    private boolean wristAngleOnTarget = false;

    private final double offsetTolerance = 7;

    public static boolean hasFoundBlock = false;


    private InterpLUT lutXOffset = new InterpLUT(); //negative values report positive y poses
    private InterpLUT lutYOffset = new InterpLUT(); //

    private Pose2d samplePoseFieldOriented;

    double bruh = 0;

    boolean isAuto;

    double autoDesiredHeading;

    ElapsedTime timer = new ElapsedTime();

    boolean isSample = false;


    public VisionToSampleInterpolate(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem,  SecondaryArmSubsystem secondarArmSubsystem, Boolean isAuto, BooleanSupplier slowMode, DoubleSupplier strafe, DoubleSupplier forward, DoubleSupplier turn, boolean isSample){
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.armSubsystem = armSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.secondaryArmSubsystem = secondarArmSubsystem;

        this.slowMode=slowMode;
        this.strafe=strafe;
        this.forward=forward;
        this.turn=turn;

        this.isAuto = isAuto;

        this.isSample = isSample;

        addRequirements(visionSubsystem, armSubsystem, intakeSubsystem);

        initializeLUTs();
    }

    public VisionToSampleInterpolate(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, SecondaryArmSubsystem secondarArmSubsystem, Boolean isAuto, BooleanSupplier slowMode, DoubleSupplier strafe, DoubleSupplier forward, DoubleSupplier turn){
        this(driveSubsystem, visionSubsystem, armSubsystem, intakeSubsystem, secondarArmSubsystem, isAuto, ()->false, ()->0, ()->0, ()->0, false);

    }

    public VisionToSampleInterpolate(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, SecondaryArmSubsystem secondarArmSubsystem,  Boolean isAuto){
        this(driveSubsystem, visionSubsystem, armSubsystem, intakeSubsystem, secondarArmSubsystem, isAuto, ()->false, ()->0, ()->0, ()->0);
    }

    private void initializeLUTs(){
//        lutXOffset.add(-555,-7.0);
//        lutXOffset.add(-525,-6.5);
//        lutXOffset.add(-495,-6.0);
//        lutXOffset.add(-464,-5.5);
        lutXOffset.add(-9999,-7.0);
        lutXOffset.add(-426,-7.0);
        lutXOffset.add(-300,-4.5);
        lutXOffset.add(-270,-4.0);
        lutXOffset.add(-240,-3.5);
        lutXOffset.add(-210,-3.0);
        lutXOffset.add(-180,-2.5);
        lutXOffset.add(-150,-2.0);
        lutXOffset.add(-125,-1.5);
        lutXOffset.add(-108,-1);
        lutXOffset.add(-53,-0.5);
        lutXOffset.add(0,0);
        lutXOffset.add(53,0.5);
        lutXOffset.add(108,1);
        lutXOffset.add(125,1.5);
        lutXOffset.add(150,2.0);
        lutXOffset.add(180,2.5);
        lutXOffset.add(210,3.0);
        lutXOffset.add(240,3.5);
        lutXOffset.add(270,4.0);
        lutXOffset.add(300,4.5);
        lutXOffset.add(426,7.0);
        lutXOffset.add(9999,7.0);
//        lutXOffset.add(464,5.5);
//        lutXOffset.add(495,6.0);
//        lutXOffset.add(525,6.5);
//        lutXOffset.add(555,7.0);



        lutYOffset.add(-350,-5.0);
        lutYOffset.add(-313,-4.25);
        lutYOffset.add(-270,-3.6);
        lutYOffset.add(-227,-2.9);
        lutYOffset.add(-185,-2.3);
        lutYOffset.add(-140,-1.7);
        lutYOffset.add(-95,-1.0);
        lutYOffset.add(-45,-0.5);
        lutYOffset.add(0,0);
        lutYOffset.add(45,0.5);
        lutYOffset.add(95,1.0);
        lutYOffset.add(140,1.5);
        lutYOffset.add(185,2.0);
        lutYOffset.add(227,2.5);
        lutYOffset.add(270,3.0);
        lutYOffset.add(313,3.5);
        lutYOffset.add(350,4.0);

        lutXOffset.createLUT();
        lutYOffset.createLUT();
    }

    @Override
    public void initialize(){
        intakeSubsystem.setDiffy(0,0);
        armSubsystem.setArmY(armReadySubIntakeY);
        hasFoundBlock=false;

//        armSubsystem.setSlideP(0.15*1.5);
        visionSubsystem.turnOnStreaming(true);
        timer.reset();
    }

    @Override
    public void execute(){
        turnpid = new BasicPID(new PIDCoefficients(kPTurn,0,0));

        Optional<RotatedRect> boxFit = Optional.empty();

        if(!hasFoundBlock) {
            boxFit = visionSubsystem.getBoxFit();
        }

        if(boxFit.isPresent()&&!hasFoundBlock&&timer.milliseconds()>200){ //REQUIRED-please raise wrist before viewing, can then work on reducing timeouts
            hasFoundBlock=true;

            List<Double> allianceOffsets = visionSubsystem.getOffsetFromBoxFit(boxFit.get());
            double xOffsetInches = lutXOffset.get(allianceOffsets.get(0));
            double yOffsetInches = lutYOffset.get(allianceOffsets.get(1));

            double allianceSkew = -visionSubsystem.getAngleFromRotatedRect(boxFit.get());

            Rotation2d allianceSkewRotation2d = new Rotation2d(Math.toRadians(allianceSkew));

            Transform2d cameraToSampleTransform = new Transform2d(new Translation2d(xOffsetInches,yOffsetInches), allianceSkewRotation2d);
            Transform2d robotToCameraTransform = new Transform2d(new Translation2d(0,armSubsystem.getCurrentX()+5), new Rotation2d());



            samplePoseFieldOriented = driveSubsystem.getPos().plus(robotToCameraTransform).plus(cameraToSampleTransform);

            intakeSubsystem.setDiffy(pitchWhenIntake, allianceSkew);

            Translation2d botToSample = samplePoseFieldOriented.relativeTo(driveSubsystem.getPos()).getTranslation();
        }

        if (hasFoundBlock){
            Translation2d botToSample = samplePoseFieldOriented.relativeTo(driveSubsystem.getPos()).getTranslation();

//            //CCW is positive
//            double headingErrorRadians  = Math.atan2(-botToSample.getX(), botToSample.getY());
//
//            double slideExtension = botToSample.getNorm();
//            double headingCalculation = turnpid.calculate(0, headingErrorRadians);
//            double turnVelocity = Math.sqrt(Math.abs(headingCalculation)) * Math.signum(headingCalculation);
////            Log.i("stupidOmega", String.valueOf(turnVelocity));
//            if(!isAuto){
//                driveSubsystem.teleDrive(slowMode, true, 10, strafe.getAsDouble(), forward.getAsDouble(), turnVelocity);
//            }
//            else{
//                driveSubsystem.pidToRotation2d(new Rotation2d(autoDesiredHeading));
//            }
////            slideExtension = MathUtils.clamp(slideExtension, 7.75, 41);
//            armSubsystem.setArmX(slideExtension);

            Log.i("bruhVisionPixelX", String.valueOf(botToSample.getX()));
            Log.i("bruhVisionPixelY", String.valueOf(botToSample.getY()));


            double desiredX = MathUtils.clamp(botToSample.getX(), -SecondaryArmSubsystem.secondaryArmLength, SecondaryArmSubsystem.secondaryArmLength);

            double yaw = Math.toDegrees(Math.acos(desiredX/SecondaryArmSubsystem.secondaryArmLength))-90;
            Log.i("bruhVisionSlideExtensionDesiredX", String.valueOf(desiredX));
            Log.i("bruhVisionSlideExtensionACosinedValue", String.valueOf(Math.toDegrees(Math.acos(desiredX/SecondaryArmSubsystem.secondaryArmLength))));



            double slideDueToYawCompensation = Math.cos(Math.toRadians(yaw))*SecondaryArmSubsystem.secondaryArmLength;

            double slideExtension = MathUtils.clamp(botToSample.getY(), ArmSubsystem.slideRetractMin, 32);
            Log.i("bruhVisionSlideExtensionPreYaw", String.valueOf(slideExtension));

            slideExtension -= slideDueToYawCompensation;

            Log.i("bruhVisionSlideExtensionPostYaw", String.valueOf(slideExtension));
            Log.i("bruhVisionSlideYawCompensation", String.valueOf(slideDueToYawCompensation));
            Log.i("bruhVisionYaw", String.valueOf(yaw));

            armSubsystem.setArmX(slideExtension);

            secondaryArmSubsystem.setDiffyYaw(-yaw);



            double wristAngle = samplePoseFieldOriented.relativeTo(driveSubsystem.getPos()).getRotation().getDegrees();
            wristAngle-=yaw;


            //Technically could remove the ifs put i think its makes it more understandable
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

            intakeSubsystem.setDiffy(-wristAngle);
        }
        else{
            if(!isAuto) {
                driveSubsystem.teleDrive(slowMode, true, 10, strafe.getAsDouble(), forward.getAsDouble(), turn.getAsDouble());
            }
        }
    }

    @Override
    public void end(boolean e){
//        armSubsystem.setSlideP(0.3);
        if(isAuto&!isSample) {
            visionSubsystem.turnOnStreaming(false);
        }
    }

    @Override
    public boolean isFinished(){
//        return false;

        Log.i("errorAutoSlides", String.valueOf(Math.abs(armSubsystem.getTargetX()-armSubsystem.getSlideX())));
        Log.i("errorAutoTarget", String.valueOf(armSubsystem.getTargetX()));
        boolean slidesOnTarget = hasFoundBlock && Math.abs(armSubsystem.getTargetX()-armSubsystem.getSlideX())<0.5;
        Log.i("errorAutoSlidesOk", String.valueOf(slidesOnTarget));


        return slidesOnTarget;
    }
}