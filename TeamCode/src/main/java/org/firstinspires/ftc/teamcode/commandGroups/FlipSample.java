package org.firstinspires.ftc.teamcode.commandGroups;

import static org.firstinspires.ftc.teamcode.other.Globals.armSubIntakeY;
import static org.firstinspires.ftc.teamcode.other.Globals.pitchIntakeWall;
import static org.firstinspires.ftc.teamcode.subSystems.SecondaryArmSubsystem.hardStoppedHighPitch;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.FullRetractSlidesUntil;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.WaitForArmCommand;
import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.SecondaryArmSubsystem;

public class FlipSample extends SequentialCommandGroup {
    public FlipSample(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, SecondaryArmSubsystem secondaryArmSubsystem) {
        addCommands(
                //tilts slides down a tad
                new InstantCommand(() -> armSubsystem.setArmY(armSubIntakeY)),
                //wait
                new WaitCommand(350),
                //grab the sample
                new InstantCommand(() -> intakeSubsystem.closeClaw()),
                //wait
                new WaitCommand(100),
                //retract slides & flip up intake
                new ParallelCommandGroup(
                        new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, 0, 0).withTimeout(100),
                        secondaryArmSubsystem.setPitchSafe(hardStoppedHighPitch),
                        new WaitForArmCommand(armSubsystem, 10, 5).withTimeout(300)
                ),
//                new WaitForSlideCommand(armSubsystem, 8,5),
                new FullRetractSlidesUntil(armSubsystem, 13.5).withTimeout(500),
//                new InstantCommand(() -> intakeSubsystem.openClaw())
                new InstantCommand(()->intakeSubsystem.clawExtraOpen())
        );
    }

    public FlipSample(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, SecondaryArmSubsystem secondaryArmSubsystem, double desiredArmAngle) {
        addCommands(
                //tilts slides down a tad
                new InstantCommand(() -> armSubsystem.setArmY(armSubIntakeY)),
                //wait
                new WaitCommand(350),
                //grab the sample
                new InstantCommand(() -> intakeSubsystem.closeClaw()),
                //wait
                new WaitCommand(100),
                //retract slides & flip up intake
                new ParallelCommandGroup(
                        new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, pitchIntakeWall, 180).withTimeout(100),
                        secondaryArmSubsystem.setPitchSafe(hardStoppedHighPitch),
                        new FullRetractSlidesUntil(armSubsystem, 13.5).withTimeout(500)
                            .andThen(new WaitForArmCommand(armSubsystem, desiredArmAngle, 5).withTimeout(300))
                ),
                new WaitCommand(50),
//                new WaitForSlideCommand(armSubsystem, 8,5),
//                new InstantCommand(() -> intakeSubsystem.openClaw())
                new InstantCommand(()->intakeSubsystem.clawExtraOpen())
        );
    }
}
