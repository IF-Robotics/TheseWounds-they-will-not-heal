package org.firstinspires.ftc.teamcode.commandGroups;

import static org.firstinspires.ftc.teamcode.other.Globals.armReadySubIntakeX;
import static org.firstinspires.ftc.teamcode.other.Globals.secondaryPitchWallIntake;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.firstWallPickUp;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.rightSideLeftSpikeFlip;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.rightSideLeftSpikePush;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.rightSideMiddleSpikeFlip;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.rightSideRightSpikeFlip;

import android.util.Log;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.commands.DriveToPointCommand;
import org.firstinspires.ftc.teamcode.commands.SecondaryArmCommand;
import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.SecondaryArmSubsystem;

public class PushSpikes extends SequentialCommandGroup {

    public PushSpikes(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, SecondaryArmSubsystem secondaryArmSubsystem){
        addCommands(
                new InstantCommand(()->driveSubsystem.enablePrecisePID(true)),
                new DriveToPointCommand(driveSubsystem, rightSideLeftSpikePush, 2, 5).withTimeout(500),
                new DriveToPointCommand(driveSubsystem, new Pose2d(51.25, -5, Rotation2d.fromDegrees(0)),2 ,5 ),
                new DriveToPointCommand(driveSubsystem, new Pose2d(51.25, -53, Rotation2d.fromDegrees(0)),2 ,5 ),
                new DriveToPointCommand(driveSubsystem, new Pose2d(51.25, -5, Rotation2d.fromDegrees(0)),2 ,5 ),
                new DriveToPointCommand(driveSubsystem, new Pose2d(61.25, -5, Rotation2d.fromDegrees(0)),2 ,5 ),
                new DriveToPointCommand(driveSubsystem, new Pose2d(61.25, -53, Rotation2d.fromDegrees(0)),2 ,5 ),
                new DriveToPointCommand(driveSubsystem, new Pose2d(61.25,-5, Rotation2d.fromDegrees(0)),2 ,5 ),
                new DriveToPointCommand(driveSubsystem, new Pose2d(65.25, -5, Rotation2d.fromDegrees(0)),2 ,5 ),
                new DriveToPointCommand(driveSubsystem, new Pose2d(65.25, -53, Rotation2d.fromDegrees(0)),2 ,5 ),








                new WaitCommand(450).andThen(new DriveToPointCommand(driveSubsystem, rightSideMiddleSpikeFlip, 2, 5).withTimeout(500))
                        .andThen(new InstantCommand(()-> {
                            Log.i("AutoStartXS2", String.valueOf(driveSubsystem.getPos().getX()));
                            Log.i("AutoStartYS2", String.valueOf(driveSubsystem.getPos().getY()));
                            Log.i("AutoStartRS2", String.valueOf(driveSubsystem.getPos().getRotation().getDegrees()));
                            Log.i("AutoTargetXS2", String.valueOf(driveSubsystem.getTargetPos().getX()));
                            Log.i("AutoTargetYS2", String.valueOf(driveSubsystem.getTargetPos().getY()));
                            Log.i("AutoTargetRS2", String.valueOf(driveSubsystem.getTargetPos().getRotation().getDegrees()));
                        })),

            new ParallelCommandGroup(
                new SequentialCommandGroup(
                        new FlipSample(armSubsystem, intakeSubsystem, secondaryArmSubsystem),
                        new IntakeSub(armSubsystem, intakeSubsystem, secondaryArmSubsystem, 155, armReadySubIntakeX-6.0),
                        new WaitCommand(800).interruptOn(()->armSubsystem.getSlideError()<0.5)
                ),
                //Wait 1000 so we dont exit boundary while we are still flipping the second sample
                new WaitCommand(1000).andThen(new DriveToPointCommand(driveSubsystem, rightSideRightSpikeFlip, 2, 5).withTimeout(500))
                        .andThen(new InstantCommand(()-> {
                            Log.i("AutoStartXS3", String.valueOf(driveSubsystem.getPos().getX()));
                            Log.i("AutoStartYS3", String.valueOf(driveSubsystem.getPos().getY()));
                            Log.i("AutoStartRS3", String.valueOf(driveSubsystem.getPos().getRotation().getDegrees()));
                            Log.i("AutoTargetXS3", String.valueOf(driveSubsystem.getTargetPos().getX()));
                            Log.i("AutoTargetYS3", String.valueOf(driveSubsystem.getTargetPos().getY()));
                            Log.i("AutoTargetRS3", String.valueOf(driveSubsystem.getTargetPos().getRotation().getDegrees()));
                        }))
            ),
            new ParallelCommandGroup(
                new FlipSample(armSubsystem, intakeSubsystem, secondaryArmSubsystem,85)
                        .andThen(new SecondaryArmCommand(secondaryArmSubsystem, secondaryPitchWallIntake))
                        .andThen(new InstantCommand(intakeSubsystem::clawExtraOpen)),
                        new WaitCommand(800)
                        .andThen(new InstantCommand(()->driveSubsystem.driveToPoint(firstWallPickUp)))
            ),
            new InstantCommand(()->driveSubsystem.enablePrecisePID(false))
        );

        addRequirements(armSubsystem, intakeSubsystem, secondaryArmSubsystem);
    }
}
