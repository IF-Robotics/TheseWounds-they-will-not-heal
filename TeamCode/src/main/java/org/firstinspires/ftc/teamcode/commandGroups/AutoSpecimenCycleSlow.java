package org.firstinspires.ftc.teamcode.commandGroups;

import static org.firstinspires.ftc.teamcode.other.Globals.armIntakeWallX;
import static org.firstinspires.ftc.teamcode.other.Globals.armIntakeWallY;
import static org.firstinspires.ftc.teamcode.other.Globals.pitchIntakeWall;
import static org.firstinspires.ftc.teamcode.other.Globals.rollIntakeWall;
import static org.firstinspires.ftc.teamcode.other.Globals.secondaryPitchWallIntake;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.highChamberRight;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.highChamberCheckpoint;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.wallPickUp;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import org.firstinspires.ftc.teamcode.commands.ArmCoordinatesCommand;
import org.firstinspires.ftc.teamcode.commands.DriveToPointCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.SecondaryArmCommand;
import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.SecondaryArmSubsystem;

public class AutoSpecimenCycleSlow extends SequentialCommandGroup {

    public AutoSpecimenCycleSlow(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, DriveSubsystem driveSubsystem, SecondaryArmSubsystem secondaryArmSubsystem) {
        addCommands(

                new SecondaryArmCommand(secondaryArmSubsystem, secondaryPitchWallIntake, 0),
                new ArmCoordinatesCommand(armSubsystem, armIntakeWallX, armIntakeWallY),
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.EXTRAOPEN, pitchIntakeWall, rollIntakeWall),

                new DriveToPointCommand(driveSubsystem, wallPickUp.plus(new Transform2d(new Translation2d(-1, 8), new Rotation2d())), 3, 3),
                new DriveToPointCommand(driveSubsystem, wallPickUp, 3, 3).withTimeout(1500),
                //wait
                new WaitCommand(100),


                // Intake specimen from wall
                new ParallelCommandGroup(
                    new RetractAfterWallIntake(armSubsystem, intakeSubsystem, secondaryArmSubsystem),
                        new WaitCommand(150)
//                            .andThen(new DriveToPointCommand(driveSubsystem, highChamberCheckpoint, 5, 5).withTimeout(1500))
                            .andThen(new DriveToPointCommand(driveSubsystem, highChamberRight, 4, 5).withTimeout(2000))
                )
        );

        addRequirements(armSubsystem, intakeSubsystem, secondaryArmSubsystem);
    }
    public AutoSpecimenCycleSlow(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, DriveSubsystem driveSubsystem, SecondaryArmSubsystem secondaryArmSubsystem, Pose2d startPos) {
        addCommands(
                new SecondaryArmCommand(secondaryArmSubsystem, secondaryPitchWallIntake, 0),
                new ArmCoordinatesCommand(armSubsystem, armIntakeWallX, armIntakeWallY),
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.EXTRAOPEN, pitchIntakeWall, rollIntakeWall),
                new DriveToPointCommand(driveSubsystem, startPos, 3, 3).withTimeout(1500),
                //wait
                new WaitCommand(100),


                // Intake specimen from wall
                new ParallelCommandGroup(
                    new RetractAfterWallIntake(armSubsystem, intakeSubsystem, secondaryArmSubsystem),
                    new WaitCommand(150)
                            .andThen(new DriveToPointCommand(driveSubsystem, highChamberCheckpoint, 5, 5).withTimeout(1500))
                            .andThen(new DriveToPointCommand(driveSubsystem, highChamberRight.transformBy(new Transform2d(new Translation2d(0, 1.0), new Rotation2d())), 4, 5).withTimeout(500))
                )
        );

        addRequirements(armSubsystem, intakeSubsystem, secondaryArmSubsystem);
    }

}
