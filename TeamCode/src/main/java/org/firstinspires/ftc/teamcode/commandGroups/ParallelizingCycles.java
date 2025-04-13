package org.firstinspires.ftc.teamcode.commandGroups;

import static org.firstinspires.ftc.teamcode.other.PosGlobals.highChamberRight;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.highChamberSpecMech;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.specMechPickUp;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.specMechPickUpCheckpoint;
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
                }),
                new ParallelCommandGroup(
                    new WaitCommand(500)
                        .andThen(new InstantCommand(()->specMechSubsystem.setArm(specArmWallIntake)))
                        .andThen(new DriveToPointCommand(driveSubsystem, specMechPickUpCheckpoint, 5, 5).withTimeout(1500))
                        .andThen(new DriveToPointCommand(driveSubsystem, specMechPickUp, 3, 5).withTimeout(1500)),
                    new RetractAfterIntake(armSubsystem, intakeSubsystem, secondaryArmSubsystem)
                        .andThen(new WaitCommand(800))
                        .andThen(new ParallelizingDropCommand(armSubsystem, intakeSubsystem, secondaryArmSubsystem))
                ),
                new WaitCommand(100),
                new ParallelCommandGroup(
                        new WaitCommand(250)
                                .andThen(new DriveToPointCommand(driveSubsystem, highChamberSpecMech, 5, 5)),
                        new SequentialCommandGroup(
                            new ParallelizingCommand(armSubsystem, intakeSubsystem, secondaryArmSubsystem, specMechSubsystem),
                            new InstantCommand(() -> armSubsystem.setArm(25)),
                            new InstantCommand(() ->intakeSubsystem.setDiffy(0,0)),
                            new InstantCommand(() ->intakeSubsystem.openClaw()),
                            new InstantCommand(() -> armSubsystem.setSlide(ArmSubsystem.slideRetractMin)),
                            new InstantCommand(() -> secondaryArmSubsystem.setDiffy(0, -30))
                        )
                ),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new WaitCommand(400),
                                new InstantCommand(() -> specMechSubsystem.openClaw()),
                                new WaitCommand(500), //we are parrallelizing so might as well use the time
                                new InstantCommand(() -> specMechSubsystem.setArm(specArmWallIntake))
                        ),
                        new SequentialCommandGroup(
                                new WaitCommand(200),
                                new LimelightToSample(driveSubsystem, armSubsystem, secondaryArmSubsystem, intakeSubsystem, limelightSubsystem).withTimeout(2000),
                                new WaitCommand(20000).interruptOn(()->Math.abs(armSubsystem.getSlideError())<0.5&&driveSubsystem.getTranslationalError()<0.5),
                                new WaitCommand(100),
                                new InstantCommand(()->driveSubsystem.enablePrecisePID(false))
                        )
                )
        );
    }
}
