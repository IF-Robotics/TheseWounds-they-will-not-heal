package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.other.Globals.armReadySubIntakeY;
import static org.firstinspires.ftc.teamcode.other.Globals.armSubIntakeY;
import static org.firstinspires.ftc.teamcode.other.Globals.pitchWhenIntake;
import static org.firstinspires.ftc.teamcode.subSystems.SecondaryArmSubsystem.extensionOffsetFromMiddle;

import android.util.Log;

import androidx.core.math.MathUtils;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.SecondaryArmSubsystem;

import java.util.Optional;

public class LimelightTeleopAimer extends CommandBase {
    ArmSubsystem armSubsystem;
    SecondaryArmSubsystem secondaryArmSubsystem;
    IntakeSubsystem intakeSubsystem;

    LimelightSubsystem limelightSubsystem;

    Optional<Pose2d> optionalResult = Optional.empty();

    Optional<Pose2d> safeResult = Optional.empty();

    ElapsedTime timer = new ElapsedTime();
    public LimelightTeleopAimer(ArmSubsystem armSubsystem, SecondaryArmSubsystem secondaryArmSubsystem, IntakeSubsystem intakeSubsystem,LimelightSubsystem limelightSubsystem){
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
        intakeSubsystem.setDiffy(-20,0);
        intakeSubsystem.openClaw();

        optionalResult = Optional.empty();
        safeResult = Optional.empty();
        timer.reset();

        limelightSubsystem.initializeCamera();

        //ONLY FOR TELEOP
//        driveSubsystem.driveToPoint(driveSubsystem.getPos());
    }

    @Override
    public void execute(){
        optionalResult = limelightSubsystem.getPose();
    }

    @Override
    public boolean isFinished(){
        if(optionalResult.isPresent() && timer.milliseconds()>200
                && armSubsystem.getArmAngle()>20 && armSubsystem.getSlideExtention()<10
        ){
            safeResult = optionalResult;
            return true;
        }
        return false;

    }

    @Override
    public void end(boolean interrupted){
        limelightSubsystem.pauseLimelight(false);
        if (safeResult.isPresent()) {
            secondaryArmSubsystem.setDiffy(0,0);
            Pose2d pose = safeResult.get();

            double desiredX = pose.getX();
            desiredX = MathUtils.clamp(desiredX, -SecondaryArmSubsystem.secondaryArmLength, SecondaryArmSubsystem.secondaryArmLength);

            double yaw = secondaryArmSubsystem.setX(desiredX);

            double slideCompensation = secondaryArmSubsystem.getSlideCompensation(yaw);

            double slideExtension = MathUtils.clamp(pose.getY()+ extensionOffsetFromMiddle + slideCompensation+0.5, ArmSubsystem.slideRetractMin,Math.min(28+slideCompensation, 28+SecondaryArmSubsystem.secondaryArmLength-2));

            armSubsystem.setArmCoordinates(slideExtension, armReadySubIntakeY);

            double angle = pose.getRotation().getDegrees();

            angle-= Math.toDegrees(yaw);

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
