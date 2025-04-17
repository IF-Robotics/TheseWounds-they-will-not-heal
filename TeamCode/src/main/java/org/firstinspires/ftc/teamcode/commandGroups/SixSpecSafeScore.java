package org.firstinspires.ftc.teamcode.commandGroups;
import static org.firstinspires.ftc.teamcode.opModes.TeleopOpMode.teleopSpec;
import static org.firstinspires.ftc.teamcode.other.Globals.armIntakeWallX;
import static org.firstinspires.ftc.teamcode.other.Globals.armIntakeWallY;
import static org.firstinspires.ftc.teamcode.other.Globals.pitchIntakeWall;
import static org.firstinspires.ftc.teamcode.other.Globals.rollIntakeWall;
import static org.firstinspires.ftc.teamcode.other.Globals.secondaryPitchWallIntake;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.firstHighChamberRight;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.firstWallPickUp;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.highChamberSafeScore;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.highChamberSpecMech;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.rightSideLeftSpikeFlip;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.specMechPickUp;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.specMechPickUpCheckpoint;

import static org.firstinspires.ftc.teamcode.other.PosGlobals.wallPickUp;
import static org.firstinspires.ftc.teamcode.subSystems.SpecMechSubsystem.specArmUp;
import static org.firstinspires.ftc.teamcode.subSystems.SpecMechSubsystem.specArmWallIntake;
import static org.firstinspires.ftc.teamcode.subSystems.SpecMechSubsystem.specAutoStart;

import android.util.Log;

import androidx.core.math.MathUtils;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.commandGroups.AutoSpecimenCycleFast;
import org.firstinspires.ftc.teamcode.commandGroups.FlipSpikes;
import org.firstinspires.ftc.teamcode.commandGroups.ParallelizingCycles;
import org.firstinspires.ftc.teamcode.commandGroups.ParallelizingDropCommand;
import org.firstinspires.ftc.teamcode.commandGroups.PushSpikes;
import org.firstinspires.ftc.teamcode.commandGroups.RetractAfterIntake;
import org.firstinspires.ftc.teamcode.commandGroups.StartSpecAuto;
import org.firstinspires.ftc.teamcode.commands.ArmCoordinatesCommand;
import org.firstinspires.ftc.teamcode.commands.DriveToPointCommand;
import org.firstinspires.ftc.teamcode.commands.DriveToPointDoubleSupplierCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.LimelightTeleopAimer;
import org.firstinspires.ftc.teamcode.commands.SecondaryArmCommand;
import org.firstinspires.ftc.teamcode.commands.WaitForSlideCommand;
import org.firstinspires.ftc.teamcode.other.AutoBase;
import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.SecondaryArmSubsystem;
public class SixSpecSafeScore extends SequentialCommandGroup {

    public SixSpecSafeScore(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, SecondaryArmSubsystem secondaryArmSubsystem, DriveSubsystem driveSubsystem) {
        addCommands(
                new InstantCommand(()->driveSubsystem.driveToPoint(wallPickUp)),
                new ParallelCommandGroup( //Positions for intaking from wall
                        secondaryArmSubsystem.setPitchSafe(secondaryPitchWallIntake),
                        new WaitForSlideCommand(armSubsystem, ArmSubsystem.slideRetractMin, 3)
                                .andThen(new ArmCoordinatesCommand(armSubsystem, armIntakeWallX, armIntakeWallY)),
                        new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.EXTRAOPEN, pitchIntakeWall, rollIntakeWall)
                ),
                new InstantCommand(()->intakeSubsystem.closeClaw()), //Moving the secondary to the score position
                new WaitCommand(50),
                //flip up the intake
                new InstantCommand(() -> intakeSubsystem.setDiffy(90,0)),
                new InstantCommand(()->secondaryArmSubsystem.setDiffyYaw(0)),
                new ArmCoordinatesCommand(armSubsystem, 0, 0), //change later  (bring the slides up)
                new DriveToPointCommand(driveSubsystem, highChamberSafeScore, 3, 5),
                new ArmCoordinatesCommand(armSubsystem, 0, 0), //change later (Bring the slides down)
                new InstantCommand(()->intakeSubsystem.openClaw())


        );
        addRequirements(armSubsystem, intakeSubsystem, secondaryArmSubsystem);
    }
    public SixSpecSafeScore(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, SecondaryArmSubsystem secondaryArmSubsystem, DriveSubsystem driveSubsystem, Pose2d startingPos) {
        addCommands(
                new InstantCommand(()->driveSubsystem.driveToPoint(startingPos)),
                new ParallelCommandGroup( //Positions for intaking from wall
                        secondaryArmSubsystem.setPitchSafe(secondaryPitchWallIntake),
                        new WaitForSlideCommand(armSubsystem, ArmSubsystem.slideRetractMin, 3)
                                .andThen(new ArmCoordinatesCommand(armSubsystem, armIntakeWallX, armIntakeWallY)),
                        new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.EXTRAOPEN, pitchIntakeWall, rollIntakeWall)
                ),
                new InstantCommand(()->intakeSubsystem.closeClaw()), //Moving the secondary to the score position
                new WaitCommand(50),
                //flip up the intake
                new InstantCommand(() -> intakeSubsystem.setDiffy(90,0)),
                new InstantCommand(()->secondaryArmSubsystem.setDiffyYaw(0)),
                new ArmCoordinatesCommand(armSubsystem, 0, 28), //change later  (bring the slides up)
                new DriveToPointCommand(driveSubsystem, highChamberSafeScore, 3, 5),
                new ArmCoordinatesCommand(armSubsystem, 0, 23), //change later (Bring the slides down)
                new InstantCommand(()->intakeSubsystem.openClaw())


        );
        addRequirements(armSubsystem, intakeSubsystem, secondaryArmSubsystem);
    }


}

