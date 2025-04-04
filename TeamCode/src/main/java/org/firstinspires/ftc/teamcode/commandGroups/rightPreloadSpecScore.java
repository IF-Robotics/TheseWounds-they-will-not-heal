package org.firstinspires.ftc.teamcode.commandGroups;

import static org.firstinspires.ftc.teamcode.other.Globals.armFrontHighChamberX;
import static org.firstinspires.ftc.teamcode.other.Globals.autoArmFrontHighChamberY;
import static org.firstinspires.ftc.teamcode.other.Globals.autoPitchFrontHighChamber;
import static org.firstinspires.ftc.teamcode.other.Globals.rollFrontHighChamber;
import static org.firstinspires.ftc.teamcode.other.Globals.secondaryPitchHighChamber;
import static org.firstinspires.ftc.teamcode.other.Globals.secondaryYawHighChamber;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.firstHighChamberRight;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.ArmCoordinatesCommand;
import org.firstinspires.ftc.teamcode.commands.DriveToPointCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.SecondaryArmCommand;
import org.firstinspires.ftc.teamcode.commands.WaitForArmCommand;
import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.SecondaryArmSubsystem;

public class rightPreloadSpecScore extends SequentialCommandGroup {

    DriveSubsystem driveSubsystem;
    IntakeSubsystem intakeSubsystem;
    ArmSubsystem armSubsystem;
    SecondaryArmSubsystem secondaryArmSubsystem;

    public rightPreloadSpecScore(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem, SecondaryArmSubsystem secondaryArmSubsystem){
        this.driveSubsystem = driveSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.armSubsystem = armSubsystem;
        this.secondaryArmSubsystem = secondaryArmSubsystem;


        addCommands(
                //raise intake and arm
                new SecondaryArmCommand(secondaryArmSubsystem, secondaryPitchHighChamber, secondaryYawHighChamber),
                new WaitForArmCommand(armSubsystem, Math.toDegrees(Math.atan2(autoArmFrontHighChamberY, armFrontHighChamberX)), 5)
                        .andThen(new ArmCoordinatesCommand(armSubsystem, armFrontHighChamberX, autoArmFrontHighChamberY)),
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, autoPitchFrontHighChamber, rollFrontHighChamber),
                // Drive to high chamber
                new DriveToPointCommand(driveSubsystem, firstHighChamberRight,5, 5).withTimeout(1500),
                //open
                new WaitCommand(100),
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.OPEN, autoPitchFrontHighChamber, rollFrontHighChamber),
                new WaitCommand(100),
                //arm to home pos
                new InstantCommand(() -> armSubsystem.setSlide(ArmSubsystem.slideRetractMin)),
                new WaitCommand(100),
                new InstantCommand(() -> armSubsystem.setArm(8))
        );

        addRequirements(intakeSubsystem, armSubsystem, secondaryArmSubsystem);
    }


}
