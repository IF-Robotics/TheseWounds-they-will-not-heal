package org.firstinspires.ftc.teamcode.commandGroups;
import static org.firstinspires.ftc.teamcode.other.Globals.armReadySubIntakeX;
import static org.firstinspires.ftc.teamcode.other.Globals.secondaryPitchWallIntake;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.firstWallPickUp;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.rightSideMiddleSpikeFlip;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.rightSideRightSpikeFlip;

import org.firstinspires.ftc.teamcode.commands.DriveToPointCommand;
import org.firstinspires.ftc.teamcode.commands.SecondaryArmCommand;
import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.SecondaryArmSubsystem;

import android.util.Log;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

public class PushSpikes extends SequentialCommandGroup{

    public PushSpikes(DriveSubsystem driveSubsystem){
        addCommands(
                new DriveToPointCommand(driveSubsystem, new Pose2d(26, -37, Rotation2d.fromDegrees(-37)), 3, 5),
                new DriveToPointCommand(driveSubsystem, new Pose2d(26, -5, Rotation2d.fromDegrees(-37)), 3, 5),
                new DriveToPointCommand(driveSubsystem, new Pose2d(32, -5, Rotation2d.fromDegrees(-37)), 3, 5),
                new DriveToPointCommand(driveSubsystem, new Pose2d(32, -37, Rotation2d.fromDegrees(-37)), 3, 5),
                new DriveToPointCommand(driveSubsystem, new Pose2d(32, -5, Rotation2d.fromDegrees(-37)), 3, 5),
                new DriveToPointCommand(driveSubsystem, new Pose2d(38, -5, Rotation2d.fromDegrees(-37)), 3, 5),
                new DriveToPointCommand(driveSubsystem, new Pose2d(38, -37, Rotation2d.fromDegrees(-37)), 3, 5)





                );

        addRequirements(driveSubsystem);
        }
}

