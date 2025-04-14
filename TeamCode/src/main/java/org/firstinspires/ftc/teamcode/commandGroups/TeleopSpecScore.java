package org.firstinspires.ftc.teamcode.commandGroups;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.wallPickUp;

import org.firstinspires.ftc.teamcode.commands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.SecondaryArmSubsystem;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

public class TeleopSpecScore extends SequentialCommandGroup{
    public TeleopSpecScore(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, SecondaryArmSubsystem secondaryArmSubsystem) {
        addCommands(
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                            new InstantCommand(() -> driveSubsystem.setStartingPos(new Pose2d(wallPickUp.getX(), wallPickUp.getY() + 2, new Rotation2d()))),
                            new AutoSpecimenCycleFast(armSubsystem,intakeSubsystem,driveSubsystem, secondaryArmSubsystem, true),
                            new AutoSpecimenCycleFast(armSubsystem,intakeSubsystem,driveSubsystem, secondaryArmSubsystem, true),
                            new AutoSpecimenCycleFast(armSubsystem,intakeSubsystem,driveSubsystem, secondaryArmSubsystem, true),
                            new AutoSpecimenCycleFast(armSubsystem,intakeSubsystem,driveSubsystem, secondaryArmSubsystem, true),
                            new AutoSpecimenCycleFast(armSubsystem,intakeSubsystem,driveSubsystem, secondaryArmSubsystem, true),
                            new AutoSpecimenCycleFast(armSubsystem,intakeSubsystem,driveSubsystem, secondaryArmSubsystem, true),
                            new AutoSpecimenCycleFast(armSubsystem,intakeSubsystem,driveSubsystem, secondaryArmSubsystem, true),
                            new AutoSpecimenCycleFast(armSubsystem,intakeSubsystem,driveSubsystem, secondaryArmSubsystem, true),
                            new AutoSpecimenCycleFast(armSubsystem,intakeSubsystem,driveSubsystem, secondaryArmSubsystem, true),
                            new AutoSpecimenCycleFast(armSubsystem,intakeSubsystem,driveSubsystem, secondaryArmSubsystem, true),
                            new AutoSpecimenCycleFast(armSubsystem,intakeSubsystem,driveSubsystem, secondaryArmSubsystem, true),
                            new AutoSpecimenCycleFast(armSubsystem,intakeSubsystem,driveSubsystem, secondaryArmSubsystem, true),
                            new AutoSpecimenCycleFast(armSubsystem,intakeSubsystem,driveSubsystem, secondaryArmSubsystem, true),
                            new AutoSpecimenCycleFast(armSubsystem,intakeSubsystem,driveSubsystem, secondaryArmSubsystem, true),
                            new AutoSpecimenCycleFast(armSubsystem,intakeSubsystem,driveSubsystem, secondaryArmSubsystem, true),
                            new AutoSpecimenCycleFast(armSubsystem,intakeSubsystem,driveSubsystem, secondaryArmSubsystem, true),
                            new AutoSpecimenCycleFast(armSubsystem,intakeSubsystem,driveSubsystem, secondaryArmSubsystem, true),
                            new AutoSpecimenCycleFast(armSubsystem,intakeSubsystem,driveSubsystem, secondaryArmSubsystem, true),
                            new AutoSpecimenCycleFast(armSubsystem,intakeSubsystem,driveSubsystem, secondaryArmSubsystem, true),
                            new AutoSpecimenCycleFast(armSubsystem,intakeSubsystem,driveSubsystem, secondaryArmSubsystem, true),
                            new AutoSpecimenCycleFast(armSubsystem,intakeSubsystem,driveSubsystem, secondaryArmSubsystem, true),
                            new AutoSpecimenCycleFast(armSubsystem,intakeSubsystem,driveSubsystem, secondaryArmSubsystem, true),
                            new AutoSpecimenCycleFast(armSubsystem,intakeSubsystem,driveSubsystem, secondaryArmSubsystem, true),
                            new AutoSpecimenCycleFast(armSubsystem,intakeSubsystem,driveSubsystem, secondaryArmSubsystem, true),
                            new AutoSpecimenCycleFast(armSubsystem,intakeSubsystem,driveSubsystem, secondaryArmSubsystem, true),
                            new AutoSpecimenCycleFast(armSubsystem,intakeSubsystem,driveSubsystem, secondaryArmSubsystem, true),
                            new AutoSpecimenCycleFast(armSubsystem,intakeSubsystem,driveSubsystem, secondaryArmSubsystem, true),
                            new AutoSpecimenCycleFast(armSubsystem,intakeSubsystem,driveSubsystem, secondaryArmSubsystem, true),
                            new AutoSpecimenCycleFast(armSubsystem,intakeSubsystem,driveSubsystem, secondaryArmSubsystem, true),
                            new AutoSpecimenCycleFast(armSubsystem,intakeSubsystem,driveSubsystem, secondaryArmSubsystem, true),
                            new AutoSpecimenCycleFast(armSubsystem,intakeSubsystem,driveSubsystem, secondaryArmSubsystem, true),
                            new AutoSpecimenCycleFast(armSubsystem,intakeSubsystem,driveSubsystem, secondaryArmSubsystem, true),
                            new AutoSpecimenCycleFast(armSubsystem,intakeSubsystem,driveSubsystem, secondaryArmSubsystem, true),
                            new AutoSpecimenCycleFast(armSubsystem,intakeSubsystem,driveSubsystem, secondaryArmSubsystem, true),
                            new AutoSpecimenCycleFast(armSubsystem,intakeSubsystem,driveSubsystem, secondaryArmSubsystem, true),
                            new AutoSpecimenCycleFast(armSubsystem,intakeSubsystem,driveSubsystem, secondaryArmSubsystem, true),
                            new AutoSpecimenCycleFast(armSubsystem,intakeSubsystem,driveSubsystem, secondaryArmSubsystem, true),
                            new AutoSpecimenCycleFast(armSubsystem,intakeSubsystem,driveSubsystem, secondaryArmSubsystem, true),
                            new AutoSpecimenCycleFast(armSubsystem,intakeSubsystem,driveSubsystem, secondaryArmSubsystem, true),
                            new AutoSpecimenCycleFast(armSubsystem,intakeSubsystem,driveSubsystem, secondaryArmSubsystem, true),
                            new AutoSpecimenCycleFast(armSubsystem,intakeSubsystem,driveSubsystem, secondaryArmSubsystem, true),
                            new AutoSpecimenCycleFast(armSubsystem,intakeSubsystem,driveSubsystem, secondaryArmSubsystem, true),
                            new AutoSpecimenCycleFast(armSubsystem,intakeSubsystem,driveSubsystem, secondaryArmSubsystem, true),
                            new AutoSpecimenCycleFast(armSubsystem,intakeSubsystem,driveSubsystem, secondaryArmSubsystem, true),
                            new AutoSpecimenCycleFast(armSubsystem,intakeSubsystem,driveSubsystem, secondaryArmSubsystem, true),
                            new AutoSpecimenCycleFast(armSubsystem,intakeSubsystem,driveSubsystem, secondaryArmSubsystem, true),
                            new AutoSpecimenCycleFast(armSubsystem,intakeSubsystem,driveSubsystem, secondaryArmSubsystem, true),
                            new AutoSpecimenCycleFast(armSubsystem,intakeSubsystem,driveSubsystem, secondaryArmSubsystem, true)
                        ),
                        new AutoDriveCommand(driveSubsystem)
                )


        );
        addRequirements(driveSubsystem, armSubsystem, intakeSubsystem);
    }
}
