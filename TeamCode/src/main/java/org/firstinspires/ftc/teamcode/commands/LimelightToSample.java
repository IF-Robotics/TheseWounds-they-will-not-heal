package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.other.Globals.armReadySubIntakeY;
import static org.firstinspires.ftc.teamcode.other.Globals.pitchWhenIntake;
import static org.firstinspires.ftc.teamcode.subSystems.SecondaryArmSubsystem.extensionOffsetFromMiddle;

import android.util.Log;

import androidx.core.math.MathUtils;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.SecondaryArmSubsystem;

import java.util.Optional;
import java.util.Timer;

public class LimelightToSample extends CommandBase {
    DriveSubsystem driveSubsystem;
    ArmSubsystem armSubsystem;
    SecondaryArmSubsystem secondaryArmSubsystem;
    IntakeSubsystem intakeSubsystem;

    LimelightSubsystem limelightSubsystem;

    Optional<Pose2d> optionalResult = Optional.empty();

    Optional<Pose2d> safeResult = Optional.empty();

    ElapsedTime timer = new ElapsedTime();
    public LimelightToSample(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, SecondaryArmSubsystem secondaryArmSubsystem, IntakeSubsystem intakeSubsystem,LimelightSubsystem limelightSubsystem){
        this.driveSubsystem = driveSubsystem;
        this.armSubsystem = armSubsystem;
        this.secondaryArmSubsystem = secondaryArmSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.limelightSubsystem = limelightSubsystem;

        addRequirements(armSubsystem, secondaryArmSubsystem, intakeSubsystem);
    }

    @Override
    public void initialize(){
        armSubsystem.setArm(25);
        armSubsystem.setSlide(ArmSubsystem.slideRetractMin);
        secondaryArmSubsystem.setDiffy(0, -30);
        intakeSubsystem.setDiffy(0,0);
        intakeSubsystem.openClaw();

        optionalResult = Optional.empty();
        safeResult = Optional.empty();
        timer.reset();

        limelightSubsystem.initializeCamera();

        //ONLY FOR TELEOP
        driveSubsystem.driveToPoint(driveSubsystem.getPos());
    }

    @Override
    public void execute(){
        optionalResult = limelightSubsystem.getPose();
    }

    @Override
    public boolean isFinished(){
        if(optionalResult.isPresent() && timer.milliseconds()>250 && armSubsystem.getArmAngle()>20){
            safeResult = optionalResult;
            return true;
        }
        return false;

    }

    @Override
    public void end(boolean interrupted){
        limelightSubsystem.pauseLimelight(false);
        if (safeResult.isPresent()) {
            driveSubsystem.enablePrecisePID(false);
            secondaryArmSubsystem.setDiffy(0,0);
            Pose2d dtPose = driveSubsystem.getPos();
            Pose2d pose = safeResult.get();

//            double heading = dtPose.getRotation().getRadians();
//            if(heading==0){heading+=0.00001;}//bruh

//            Pose2d transformedPose = new Pose2d(
//                    dtPose.getX() + Math.cos(heading)*pose.getX()*Math.signum(heading)*-1,
//                    dtPose.getY() + Math.sin(heading)*pose.getX(),`
//                    dtPose.getRotation()
//            );


            Pose2d transformedPose = new Pose2d(
                dtPose.getX()+pose.getX(),
                dtPose.getY(),
                dtPose.getRotation()
            );

            driveSubsystem.driveToPoint(transformedPose);

            double slideExtension = MathUtils.clamp(pose.getY()+extensionOffsetFromMiddle, ArmSubsystem.slideRetractMin, 30);
            armSubsystem.setArmCoordinates(slideExtension, armReadySubIntakeY+0.5);

            double angle = pose.getRotation().getDegrees();

            if(angle > 90){
                while (angle > 90){
                    angle-=180;
                }
            }
            if(angle<-90){
                while(angle<-90){
                    angle+=180;
                }
            }

            Log.i("anglePost", String.valueOf(angle));

            intakeSubsystem.setDiffy(pitchWhenIntake+20,angle);
        }
    }

}
