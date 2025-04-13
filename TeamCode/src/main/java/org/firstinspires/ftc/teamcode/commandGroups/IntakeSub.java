package org.firstinspires.ftc.teamcode.commandGroups;

import static org.firstinspires.ftc.teamcode.other.Globals.armReadySubIntakeX;
import static org.firstinspires.ftc.teamcode.other.Globals.armReadySubIntakeY;
import static org.firstinspires.ftc.teamcode.other.Globals.pitchWhenIntake;
import static org.firstinspires.ftc.teamcode.other.Globals.rollWhenIntake;

import android.util.Log;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.ArmCoordinatesCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.SecondaryArmCommand;
import org.firstinspires.ftc.teamcode.commands.WaitForArmCommand;
import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.SecondaryArmSubsystem;

public class IntakeSub extends SequentialCommandGroup {
    public IntakeSub(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, SecondaryArmSubsystem secondaryArmSubsystem){
        addCommands(
            new InstantCommand(()->intakeSubsystem.resetRotateIntake()),
            new InstantCommand(()->secondaryArmSubsystem.setDiffyYaw(0)),
            //reset intake array
            new InstantCommand(() -> intakeSubsystem.setDiffy(0,0)),
            //move intake down
            new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.OPEN, 0, rollWhenIntake),
            secondaryArmSubsystem.intakeSub(),
            //wait for arm to be horizontal
            new WaitForArmCommand(armSubsystem, 8, 5),
            //arm & intake to correct pos
            new ArmCoordinatesCommand(armSubsystem, armReadySubIntakeX, armReadySubIntakeY),
            new WaitCommand(250),
            new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.OPEN, pitchWhenIntake, rollWhenIntake)
        );

        addRequirements(armSubsystem, intakeSubsystem);
    }

    //only for auto so we can optimize it
    public IntakeSub(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, SecondaryArmSubsystem secondaryArmSubsystem, double roll, double subIntakeDistance){
        addCommands(
                //move intake down
                secondaryArmSubsystem.intakeSub(),
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.OPEN, pitchWhenIntake, roll),
                //wait for arm to be horizontal
                new WaitForArmCommand(armSubsystem, 10, 15).withTimeout(300),
                //arm & intake to correct pos
                new ArmCoordinatesCommand(armSubsystem, subIntakeDistance, armReadySubIntakeY+2.5)
        );

        addRequirements(armSubsystem, intakeSubsystem);
    }
}
