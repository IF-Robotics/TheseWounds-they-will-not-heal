package org.firstinspires.ftc.teamcode.commandGroups;

import static org.firstinspires.ftc.teamcode.other.PosGlobals.highChamberRight;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.specMechPickUp;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.wallPickUp;
import static org.firstinspires.ftc.teamcode.subSystems.SpecMechSubsystem.specArmWallIntake;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import org.firstinspires.ftc.teamcode.commands.DriveToPointCommand;
import org.firstinspires.ftc.teamcode.commands.LimelightToSample;
import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.SecondaryArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.SpecMechSubsystem;

public class ParallelizingCycles extends SequentialCommandGroup {
    public ParallelizingCycles(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, SecondaryArmSubsystem secondaryArmSubsystem, SpecMechSubsystem specMechSubsystem, LimelightSubsystem limelightSubsystem){
        addCommands(
                new InstantCommand(()->{
                    specMechSubsystem.openClaw();
                    specMechSubsystem.setArm(specArmWallIntake);
                }),
                new ParallelCommandGroup(
                    new DriveToPointCommand(driveSubsystem, specMechPickUp.plus(new Transform2d(new Translation2d(0, 8), new Rotation2d())), 3, 5)
                        .andThen(new DriveToPointCommand(driveSubsystem, specMechPickUp, 3, 5).withTimeout(1500)),
                    new ParallelizingDropCommand(armSubsystem, intakeSubsystem, secondaryArmSubsystem)
                ),
                new WaitCommand(100),
                new ParallelCommandGroup(
                        new WaitCommand(250)
                                .andThen(new DriveToPointCommand(driveSubsystem, highChamberRight, 5, 5)),
                        new ParallelizingCommand(armSubsystem, intakeSubsystem, secondaryArmSubsystem, specMechSubsystem)
                ),
                new LimelightToSample(driveSubsystem, armSubsystem, secondaryArmSubsystem, intakeSubsystem, limelightSubsystem),
                new WaitCommand(1000),
                new RetractAfterIntake(armSubsystem, intakeSubsystem, secondaryArmSubsystem)
        );
    }
}
