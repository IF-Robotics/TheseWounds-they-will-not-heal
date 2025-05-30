package org.firstinspires.ftc.teamcode.commandGroups;

import static org.firstinspires.ftc.teamcode.other.Globals.*;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.*;

import android.util.Log;

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

public class AutoSpecimenCycleFast extends SequentialCommandGroup {
    public AutoSpecimenCycleFast(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, DriveSubsystem driveSubsystem, SecondaryArmSubsystem secondaryArmSubsystem) {
            addCommands(
                new SequentialCommandGroup(
                    new InstantCommand(()->armSubsystem.setSlidePower(1.00)),
                    new ParallelCommandGroup(
                        new DriveToPointCommand(driveSubsystem, wallPickUpFastCheckpoint, 5, 5).withTimeout(1500)
                                .andThen(new DriveToPointCommand(driveSubsystem, wallPickUp, 5, 5).withTimeout(1500)),
//                        new DriveToPointCommand(driveSubsystem, wallPickUp, 5, 5).withTimeout(1500),
                        new WaitCommand(250)
                                .andThen(secondaryArmSubsystem.setPitchSafe(secondaryPitchWallIntake)),
                        new InstantCommand(()->intakeSubsystem.clawExtraOpen()),
                        new WaitForSlideCommand(armSubsystem, ArmSubsystem.slideRetractMin, 3)
                            .andThen(new ArmCoordinatesCommand(armSubsystem, armIntakeWallX, armIntakeWallY))
                            .andThen(new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.EXTRAOPEN, pitchIntakeWall, rollIntakeWall))
                            .andThen(new InstantCommand(()->armSubsystem.nautilusUp()))
                    ),

                    // Intake specimen from wall
                    new ParallelCommandGroup(
                            new SequentialCommandGroup(

                                    //grab the specimen
                                    new InstantCommand(()->intakeSubsystem.closeClaw()),
                                    new WaitCommand(50),
                                    //flip up the intake
                                    new InstantCommand(()->armSubsystem.setSlide(12)),
                                    secondaryArmSubsystem.setPitchSafe(10),
                                    new WaitForArmCommand(armSubsystem, 30, 10).withTimeout(500),
                                    new InstantCommand(()->armSubsystem.setSlidePower(0.75)),
                                    new ParallelCommandGroup(
                                            new WaitCommand(1500).interruptOn(()->driveSubsystem.getPos().getX()<35)
                                                    .andThen(new ArmCoordinatesCommand(armSubsystem, armFastX, armFastY)), //wait for secondary arm yaw to clear nautilus
                                            new WaitCommand(50).andThen(new InstantCommand(()->secondaryArmSubsystem.setDiffyYaw(5))), //wait for sample to rotate
                                            new WaitCommand(500),
                                            new InstantCommand(()->intakeSubsystem.setDiffy(50, 0))
                                    )
                            ),

                            new SequentialCommandGroup(
                                    new WaitCommand(50),
//                                    new InstantCommand(()->driveSubsystem.enablePrecisePID(false)),
//                                    new DriveToPointCommand(driveSubsystem, highChamberFastCheckpoint, 5, 5).withTimeout(1500),
                                    new DriveToPointCommand(driveSubsystem, highChamberFast, 4, 5).withTimeout(2200),
                                    new WaitCommand(200)
//                                    new InstantCommand(()->armSubsystem.nautilusDown()),
//                                    new InstantCommand(()->driveSubsystem.enablePrecisePID(false))
                            )
                    )
                )
            );

            addRequirements(armSubsystem, intakeSubsystem, secondaryArmSubsystem);
    }

    public AutoSpecimenCycleFast(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, DriveSubsystem driveSubsystem, SecondaryArmSubsystem secondaryArmSubsystem, Pose2d startingPos) {
        addCommands(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new DriveToPointCommand(driveSubsystem, startingPos, 5, 5).withTimeout(1500),
                                secondaryArmSubsystem.setPitchSafe(secondaryPitchWallIntake),
                                new WaitForSlideCommand(armSubsystem, ArmSubsystem.slideRetractMin, 3)
                                        .andThen(new ArmCoordinatesCommand(armSubsystem, armIntakeWallX, armIntakeWallY)),
                                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.EXTRAOPEN, pitchIntakeWall, rollIntakeWall),
                                new InstantCommand(()->armSubsystem.nautilusUp())
                        ),
                        new WaitCommand(00),
                        // Intake specimen from wall
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        //grab the specimen
                                        new InstantCommand(()->intakeSubsystem.closeClaw()),
                                        new WaitCommand(50),
                                        //flip up the intake
                                        new InstantCommand(()->armSubsystem.setSlide(12)),
                                        secondaryArmSubsystem.setPitchSafe(10),
                                        new WaitForArmCommand(armSubsystem, 30, 10).withTimeout(500),
                                        new ParallelCommandGroup(
                                                new WaitCommand(1500).interruptOn(()->driveSubsystem.getPos().getX()<33)
                                                        .andThen(new ArmCoordinatesCommand(armSubsystem, armFastX, armFastY)), //wait for secondary arm yaw to clear nautilus
                                                new WaitCommand(50).andThen(new InstantCommand(()->secondaryArmSubsystem.setDiffyYaw(5))), //wait for sample to rotate
                                                new WaitCommand(500),
                                                new InstantCommand(()->intakeSubsystem.setDiffy(50, 0))
                                        )
                                ),

                                new SequentialCommandGroup(
                                    new WaitCommand(50),
//                                    new InstantCommand(()->driveSubsystem.enablePrecisePID(false)),
                                    //checkpoint is just pickup pos
                                    new DriveToPointCommand(driveSubsystem, wallPickUp, 5, 5).withTimeout(1500),
                                    new DriveToPointCommand(driveSubsystem, highChamberFast, 4, 5).withTimeout(2200),
                                    new WaitCommand(100)
//                                    new InstantCommand(()->armSubsystem.nautilusDown()),
//                                    new InstantCommand(()->driveSubsystem.enablePrecisePID(false))
                                )
                        )
                )
        );

        addRequirements(armSubsystem, intakeSubsystem, secondaryArmSubsystem);
    }

    public AutoSpecimenCycleFast(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, DriveSubsystem driveSubsystem, SecondaryArmSubsystem secondaryArmSubsystem, boolean isTeleop) {
        addCommands(
                new SequentialCommandGroup(
                        new InstantCommand(()->armSubsystem.setSlidePower(1.00)),
                        new ParallelCommandGroup(
                                new DriveToPointCommand(driveSubsystem, wallPickUpFastCheckpoint, 5, 5).withTimeout(1500)
                                        .andThen(new DriveToPointCommand(driveSubsystem, wallPickUp, 5, 5).withTimeout(1500)),
//                        new DriveToPointCommand(driveSubsystem, wallPickUp, 5, 5).withTimeout(1500),
                                new WaitCommand(400)
                                        .andThen(secondaryArmSubsystem.setPitchSafe(secondaryPitchWallIntake)),
                                new InstantCommand(()->intakeSubsystem.clawExtraOpen()),
                                new WaitForSlideCommand(armSubsystem, ArmSubsystem.slideRetractMin, 3)
                                        .andThen(new ArmCoordinatesCommand(armSubsystem, armIntakeWallX, armIntakeWallY))
                                        .andThen(new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.EXTRAOPEN, pitchIntakeWall, rollIntakeWall))
                                        .andThen(new InstantCommand(()->armSubsystem.nautilusUp()))
                        ),
                        new WaitCommand(200),

                        // Intake specimen from wall
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(

                                        //grab the specimen
                                        new InstantCommand(()->intakeSubsystem.closeClaw()),
                                        new WaitCommand(50),
                                        //flip up the intake
                                        new InstantCommand(()->armSubsystem.setSlide(12)),
                                        secondaryArmSubsystem.setPitchSafe(10),
                                        new WaitForArmCommand(armSubsystem, 30, 10).withTimeout(500),
                                        new InstantCommand(()->armSubsystem.setSlidePower(0.75)),
                                        new ParallelCommandGroup(
                                                new WaitCommand(1500).interruptOn(()->driveSubsystem.getPos().getX()<35)
                                                        .andThen(new ArmCoordinatesCommand(armSubsystem, armFastX, armFastY)), //wait for secondary arm yaw to clear nautilus
                                                new WaitCommand(50).andThen(new InstantCommand(()->secondaryArmSubsystem.setDiffyYaw(5))), //wait for sample to rotate
                                                new WaitCommand(500),
                                                new InstantCommand(()->intakeSubsystem.setDiffy(50, 0))
                                        )
                                ),

                                new SequentialCommandGroup(
                                        new WaitCommand(50),
//                                    new InstantCommand(()->driveSubsystem.enablePrecisePID(false)),
//                                    new DriveToPointCommand(driveSubsystem, highChamberFastCheckpoint, 5, 5).withTimeout(1500),
                                        new DriveToPointCommand(driveSubsystem, highChamberFast, 4, 5).withTimeout(2200),
                                        new WaitCommand(300)
//                                    new InstantCommand(()->armSubsystem.nautilusDown()),
//                                    new InstantCommand(()->driveSubsystem.enablePrecisePID(false))
                                )
                        )
                )
        );

        addRequirements(armSubsystem, intakeSubsystem, secondaryArmSubsystem);
    }
}
