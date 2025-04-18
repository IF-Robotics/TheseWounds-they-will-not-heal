package org.firstinspires.ftc.teamcode.commandGroups;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import static org.firstinspires.ftc.teamcode.other.Globals.*;

import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.SecondaryArmCommand;
import org.firstinspires.ftc.teamcode.commands.WaitForArmCommand;
import org.firstinspires.ftc.teamcode.commands.WaitForSlideCommand;
import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.ColorSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.SecondaryArmSubsystem;

public class RetractAfterIntake extends SequentialCommandGroup{

    public RetractAfterIntake(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, SecondaryArmSubsystem secondaryArmSubsystem){
        addCommands(
                //tilts slides down a tad
                new InstantCommand(()->intakeSubsystem.setPitch(pitchWhenIntake)),
                new InstantCommand(() -> armSubsystem.setArmY(armSubIntakeY)),
                //wait
                new WaitCommand(300),
                //grab the sample
                new InstantCommand(() -> intakeSubsystem.closeClaw()),
                //wait
                new WaitCommand(150),
                new InstantCommand(()->intakeSubsystem.setDiffy(0,90)),
                new InstantCommand(()-> armSubsystem.setArm(5)),
                new ParallelCommandGroup(
                    //retract slides
                    new WaitForSlideCommand(armSubsystem, 8, 5),
                    //move intake out of the way
                    secondaryArmSubsystem.setPitchYawSafe(0,0)
                )
        );

        addRequirements(armSubsystem, intakeSubsystem);
    }


    //Im overloading it, the boolean has no impacgt, who gaf anymore
    public RetractAfterIntake(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, ColorSubsystem colorSubsystem, SecondaryArmSubsystem secondaryArmSubsystem){
        Command dropOppositeAllianceSample = new ConditionalCommand(
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.OPEN, 0, 0)
                        .andThen(new InstantCommand(()->cancelAndReextend(intakeSubsystem, armSubsystem, secondaryArmSubsystem))),
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, 0, 0),
                ()->colorSubsystem.holdingOppositeColor()
        );
        addCommands(
                //tilts slides down a tad
                new InstantCommand(() -> armSubsystem.setArmY(armSubIntakeY)),
                //wait
                new WaitCommand(300),
                //grab the sample
                new InstantCommand(() -> intakeSubsystem.closeClaw()),
                //wait
                new WaitCommand(100),

                dropOppositeAllianceSample,

                new InstantCommand(()-> armSubsystem.setArm(15)),
                new ParallelDeadlineGroup(
                        //retract slides
                        new WaitForSlideCommand(armSubsystem, 8, 15),
                        //move intake out of the way
                        new ConditionalCommand(
                                new SecondaryArmCommand(secondaryArmSubsystem, 45, 0),
                                new SequentialCommandGroup(
                                        new SecondaryArmCommand(secondaryArmSubsystem, 0),
                                        new WaitCommand(500),
                                        new SecondaryArmCommand(secondaryArmSubsystem, 45, 0)
                                ),
                                () -> Math.abs(secondaryArmSubsystem.getYawAngle()) < 5
                        )
                )
        );




        addRequirements(armSubsystem, intakeSubsystem);
    }



    //retract after intaking then ready to dropoff to observation zone
    public RetractAfterIntake(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, SecondaryArmSubsystem secondaryArmSubsystem, Boolean specMode){
        addCommands(
                new InstantCommand(()->intakeSubsystem.setPitch(pitchWhenIntake)),
                //tilts slides down a tad
                new InstantCommand(() -> armSubsystem.setArmY(armSubIntakeY)),
                //wait
                new WaitCommand(300),
                //grab the sample
                new InstantCommand(() -> intakeSubsystem.closeClaw()),
                //wait
                new WaitCommand(150),
                new InstantCommand(()->intakeSubsystem.setDiffy(0,0)), //maybe set roll to 90
                new InstantCommand(()-> armSubsystem.setArm(5)),
                new ParallelCommandGroup(
                    secondaryArmSubsystem.setPitchYawSafe(0,0),

                    //retract slides
                    new WaitForSlideCommand(armSubsystem, 8, 15)
                ),
                //raise arm
                new WaitForArmCommand(armSubsystem, 80, 45),
                //move secondaryArm to the side, wait until above the side plates
                new InstantCommand(()-> secondaryArmSubsystem.setDiffyYaw(90))
        );

        addRequirements(armSubsystem, intakeSubsystem);
    }


    public void cancelAndReextend(IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem, SecondaryArmSubsystem secondaryArmSubsystem){
        this.cancel();
        CommandScheduler.getInstance().schedule(new IntakeSub(armSubsystem, intakeSubsystem, secondaryArmSubsystem));
    }

    public RetractAfterIntake(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, SecondaryArmSubsystem secondaryArmSubsystem, boolean isAuto){
        addCommands(
                //tilts slides down a tad
                new InstantCommand(()->intakeSubsystem.setPitch(pitchWhenIntake)),
                new InstantCommand(() -> armSubsystem.setArmY(armSubIntakeY)),
                //wait
                new WaitCommand(400),
                //grab the sample
                new InstantCommand(() -> intakeSubsystem.closeClaw()),
                //wait
                new WaitCommand(250),
                new InstantCommand(()->intakeSubsystem.setDiffy(0,90)),
                new InstantCommand(()-> armSubsystem.setArm(5)),
                new ParallelCommandGroup(
                        //retract slides
                        new WaitForSlideCommand(armSubsystem, 8, 5),
                        //move intake out of the way
                        secondaryArmSubsystem.setPitchYawSafe(0,0)
                )
        );

        addRequirements(armSubsystem, intakeSubsystem);
    }

}
