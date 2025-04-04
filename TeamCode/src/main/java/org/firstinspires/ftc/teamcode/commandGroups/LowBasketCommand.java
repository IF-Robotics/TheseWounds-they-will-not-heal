package org.firstinspires.ftc.teamcode.commandGroups;

import static org.firstinspires.ftc.teamcode.other.Globals.armHighBasketX;
import static org.firstinspires.ftc.teamcode.other.Globals.armHighBasketY;
import static org.firstinspires.ftc.teamcode.other.Globals.armLowBasketY;
import static org.firstinspires.ftc.teamcode.other.Globals.pitchWhenBasket;
import static org.firstinspires.ftc.teamcode.other.Globals.rollWhenBasket;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.ArmCoordinatesCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.SecondaryArmCommand;
import org.firstinspires.ftc.teamcode.commands.WaitForArmCommand;
import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.SecondaryArmSubsystem;

public class LowBasketCommand extends SequentialCommandGroup{

    public LowBasketCommand(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, SecondaryArmSubsystem secondaryArmSubsystem){
        addCommands(
                //move arm back
                new WaitForArmCommand(armSubsystem, 100, 45),

                //move to high basket
                new ArmCoordinatesCommand(armSubsystem, armHighBasketX, armLowBasketY),
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, pitchWhenBasket, rollWhenBasket),
                new SecondaryArmCommand(secondaryArmSubsystem, 30, 0)


        );

        addRequirements(armSubsystem, intakeSubsystem);
    }
}
