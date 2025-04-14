package org.firstinspires.ftc.teamcode.commandGroups;

import static org.firstinspires.ftc.teamcode.other.Globals.*;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.*;

import org.firstinspires.ftc.teamcode.commands.*;
import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.SecondaryArmSubsystem;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

public class AutoSpecimenCycleFastOld extends SequentialCommandGroup {
    public AutoSpecimenCycleFastOld(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, DriveSubsystem driveSubsystem, SecondaryArmSubsystem secondaryArmSubsystem) {
        addCommands(
                new SequentialCommandGroup(
                        new ArmCoordinatesCommand(armSubsystem, armIntakeWallX, armIntakeWallY),
                        new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.EXTRAOPEN, pitchIntakeWall, rollIntakeWall),
                        new InstantCommand(()->armSubsystem.nautilusUp()),

                        new ParallelCommandGroup(
                                new DriveToPointCommand(driveSubsystem, wallPickUpFastCheckpoint, 5, 5).withTimeout(1500)
                                        .andThen(new DriveToPointCommand(driveSubsystem, wallPickUp, 3, 3).withTimeout(1500)),
                                secondaryArmSubsystem.setPitchSafe(secondaryPitchWallIntake),
                                new ArmCoordinatesCommand(armSubsystem, armIntakeWallX, armIntakeWallY),
                                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.EXTRAOPEN, pitchIntakeWall, rollIntakeWall),
                                new InstantCommand(()->armSubsystem.nautilusUp())
                        ),

                        // Intake specimen from wall
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        //grab the specimen
                                        new InstantCommand(()->intakeSubsystem.closeClaw()),
                                        new WaitCommand(50),
                                        //flip up the intake
                                        new InstantCommand(()->armSubsystem.setSlide(12)),
                                        secondaryArmSubsystem.setPitchSafe(0),
                                        new WaitForArmCommand(armSubsystem, 30, 15).withTimeout(500),
                                        new ParallelCommandGroup(
                                                new ArmCoordinatesCommand(armSubsystem, armFastX, armFastY), //wait for secondary arm yaw to clear nautilus
                                                new WaitCommand(50).andThen(new InstantCommand(()->secondaryArmSubsystem.setDiffyYaw(5))), //wait for sample to rotate
                                                new InstantCommand(()->intakeSubsystem.setDiffy(50, 0))
                                        )
                                ),

                                new SequentialCommandGroup(
                                        new WaitCommand(50),
                                        new InstantCommand(()->driveSubsystem.enablePrecisePID(true)),
//                                    new DriveToPointCommand(driveSubsystem, highChamberFastCheckpoint, 10, 5).withTimeout(1500),
                                        new DriveToPointCommand(driveSubsystem, highChamberFast, 4, 5).withTimeout(2000),
//                                    new InstantCommand(()->armSubsystem.nautilusDown()),
                                        new InstantCommand(()->driveSubsystem.enablePrecisePID(false))
                                )
                        )
                )
        );

        addRequirements(armSubsystem, intakeSubsystem, secondaryArmSubsystem, driveSubsystem);
    }
}