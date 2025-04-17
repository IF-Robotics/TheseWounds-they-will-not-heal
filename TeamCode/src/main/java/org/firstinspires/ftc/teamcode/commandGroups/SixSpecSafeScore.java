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
import static org.firstinspires.ftc.teamcode.other.PosGlobals.highChamberSafeScoreCheckpoint;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.highChamberSpecMech;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.rightSideLeftSpikeFlip;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.specMechPickUp;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.specMechPickUpCheckpoint;

import static org.firstinspires.ftc.teamcode.other.PosGlobals.wallPickUp;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.wallPickUpSafe;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.wallPickUpSafeCheckpoint;
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
import org.firstinspires.ftc.teamcode.commands.WaitForArmCommand;
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
                new ParallelCommandGroup( //Positions for intaking from wall
                        new DriveToPointCommand(driveSubsystem, wallPickUpSafeCheckpoint, 5, 5).withTimeout(1500)
                                .andThen(new WaitCommand(200))
                                .andThen(new DriveToPointCommand(driveSubsystem,wallPickUpSafe,5,5).withTimeout(500)),
                        new ArmCoordinatesCommand(armSubsystem, 8,10),
                        new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.OPEN, -30, 180),
                        new InstantCommand(()->secondaryArmSubsystem.setDiffyPitch(0))
                ),
                new InstantCommand(()->intakeSubsystem.closeClaw()), //Moving the secondary to the score position
                new WaitCommand(50),
                //flip up the intake
                new InstantCommand(() -> intakeSubsystem.setDiffy(80,0)),
                new InstantCommand(()->secondaryArmSubsystem.setDiffy(35,0)),
                new ParallelCommandGroup(
                        new DriveToPointCommand(driveSubsystem, highChamberSafeScore, 5, 5),
                        new WaitForArmCommand(armSubsystem, 90, 5)
                            .andThen(new ArmCoordinatesCommand(armSubsystem, 0, 22))
                ),
                new ArmCoordinatesCommand(armSubsystem, 0, 13), //change later (Bring the slides down)
                new WaitCommand(300),
                new InstantCommand(()->intakeSubsystem.openClaw())


        );
        addRequirements(armSubsystem, intakeSubsystem, secondaryArmSubsystem);
    }
    public SixSpecSafeScore(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, SecondaryArmSubsystem secondaryArmSubsystem, DriveSubsystem driveSubsystem, Pose2d startingPos) {
        addCommands(
                new ParallelCommandGroup( //Positions for intaking from wall
                        new DriveToPointCommand(driveSubsystem,startingPos,5,5).withTimeout(1500),
                        new ArmCoordinatesCommand(armSubsystem, 8,10),
                        new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.OPEN, -30, 180)
                ),
                new InstantCommand(()->Log.i("safespectargetposx", String.valueOf(driveSubsystem.getTargetPos().getX()))),
                new InstantCommand(()->Log.i("safespectargetposy", String.valueOf(driveSubsystem.getTargetPos().getY()))),
                new InstantCommand(()->Log.i("safespectargetposheading", String.valueOf(driveSubsystem.getTargetPos().getRotation().getDegrees()))),
                new InstantCommand(()->intakeSubsystem.closeClaw()), //Moving the secondary to the score position
                new WaitCommand(50),
                //flip up the intake
                new InstantCommand(() -> intakeSubsystem.setDiffy(80,0)),
                new InstantCommand(()->secondaryArmSubsystem.setDiffy(35, 0)),
                new ParallelCommandGroup(
                    new DriveToPointCommand(driveSubsystem, highChamberSafeScoreCheckpoint, 5, 5)
                        .andThen(new DriveToPointCommand(driveSubsystem, highChamberSafeScore, 5, 5)),
                    new WaitForArmCommand(armSubsystem, 90, 5)
                        .andThen(new ArmCoordinatesCommand(armSubsystem, 0, 22))
                ),
                new InstantCommand(()->Log.i("safespectargetposx", String.valueOf(driveSubsystem.getTargetPos().getX()))),
                new InstantCommand(()->Log.i("safespectargetposy", String.valueOf(driveSubsystem.getTargetPos().getY()))),
                new InstantCommand(()->Log.i("safespectargetposheading", String.valueOf(driveSubsystem.getTargetPos().getRotation().getDegrees()))),

                new ArmCoordinatesCommand(armSubsystem, 0, 13), //change later (Bring the slides down)
                new WaitCommand(300),
                new InstantCommand(()->intakeSubsystem.openClaw())


        );
        addRequirements(armSubsystem, intakeSubsystem, secondaryArmSubsystem);
    }


}

