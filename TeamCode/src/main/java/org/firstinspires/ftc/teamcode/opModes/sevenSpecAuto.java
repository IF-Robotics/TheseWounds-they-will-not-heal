package org.firstinspires.ftc.teamcode.opModes;
import static org.firstinspires.ftc.teamcode.opModes.TeleopOpMode.teleopSpec;
import static org.firstinspires.ftc.teamcode.other.Globals.*;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.*;
import static org.firstinspires.ftc.teamcode.subSystems.SpecMechSubsystem.specArmUp;
import static org.firstinspires.ftc.teamcode.subSystems.SpecMechSubsystem.specArmWallIntake;
import static org.firstinspires.ftc.teamcode.subSystems.SpecMechSubsystem.specAutoStart;

import android.util.Log;

import androidx.core.math.MathUtils;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.commandGroups.AutoSpecimenCycleFast;
import org.firstinspires.ftc.teamcode.commandGroups.AutoSpecimenCycleSlow;
import org.firstinspires.ftc.teamcode.commandGroups.DropCommand;
import org.firstinspires.ftc.teamcode.commandGroups.DropOffCommand;
import org.firstinspires.ftc.teamcode.commandGroups.FlipSpikes;
import org.firstinspires.ftc.teamcode.commandGroups.IntakeSub;
import org.firstinspires.ftc.teamcode.commandGroups.ParallelizingCycles;
import org.firstinspires.ftc.teamcode.commandGroups.ParallelizingDropCommand;
import org.firstinspires.ftc.teamcode.commandGroups.RetractAfterIntake;
import org.firstinspires.ftc.teamcode.commandGroups.StartSpecAuto;
import org.firstinspires.ftc.teamcode.commandGroups.SweepSpikes;
import org.firstinspires.ftc.teamcode.commands.ArmCoordinatesCommand;
import org.firstinspires.ftc.teamcode.commands.DriveToPointCommand;
import org.firstinspires.ftc.teamcode.commands.DriveToPointDoubleSupplierCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.LimelightTeleopAimer;
import org.firstinspires.ftc.teamcode.commands.LimelightToSample;
import org.firstinspires.ftc.teamcode.commands.SecondaryArmCommand;
import org.firstinspires.ftc.teamcode.commands.VisionToSampleInterpolate;
import org.firstinspires.ftc.teamcode.commands.WaitForArmCommand;
import org.firstinspires.ftc.teamcode.commands.WaitForSlideCommand;
import org.firstinspires.ftc.teamcode.commands.holdDTPosCommand;
import org.firstinspires.ftc.teamcode.other.AutoBase;
import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.SecondaryArmSubsystem;

@Autonomous(name="7+0")

public class sevenSpecAuto extends AutoBase {

    private double subX1 = 0;
    private double subY1 = 7.5;

    private double subX2 = 0;
    private double subY2 = 7.5;

    int currentSubIndex = 1;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    @Override
    public void initialize() {
        super.initialize();

        intakeSubsystem.setDiffy(-20,0);
        secondaryArmSubsystem.setDiffy(90, 0);

        specMechSubsystem.closeClaw();
        specMechSubsystem.setArm(specAutoStart);

        limelightSubsystem.initializeCamera();


        schedule(new SequentialCommandGroup(
                new StartSpecAuto(driveSubsystem, armSubsystem, intakeSubsystem, secondaryArmSubsystem),

                new InstantCommand(()-> {
                    Log.i("AutoStartX", String.valueOf(driveSubsystem.getPos().getX()));
                    Log.i("AutoStartY", String.valueOf(driveSubsystem.getPos().getY()));
                    Log.i("AutoStartR", String.valueOf(driveSubsystem.getPos().getRotation().getDegrees()));
                    Log.i("AutoTargetX", String.valueOf(driveSubsystem.getTargetPos().getX()));
                    Log.i("AutoTargetY", String.valueOf(driveSubsystem.getTargetPos().getY()));
                    Log.i("AutoTargetR", String.valueOf(driveSubsystem.getTargetPos().getRotation().getDegrees()));
                }),

                new ParallelCommandGroup(
                        new InstantCommand(() -> specMechSubsystem.closeClaw()),
                        new InstantCommand(() -> specMechSubsystem.setArm(specArmUp)),
                        new InstantCommand(() -> armSubsystem.setArm(25)),
                        new InstantCommand(() -> armSubsystem.setSlide(ArmSubsystem.slideRetractMin)),
                        new InstantCommand(() -> secondaryArmSubsystem.setDiffy(-20, 0)),
                        new InstantCommand(() ->intakeSubsystem.setDiffy(0,0)),
                        new InstantCommand(() ->intakeSubsystem.openClaw())
                ),
                //servo nudge issue
                new InstantCommand(()->{
                    secondaryArmSubsystem.setDiffyPitch(SecondaryArmSubsystem.hardStoppedHighPitch);
                    specMechSubsystem.closeClaw();
                }),
                new WaitCommand(50),
                new InstantCommand(()->secondaryArmSubsystem.setDiffyPitch(0)),
                new WaitCommand(100),

                new DriveToPointDoubleSupplierCommand(driveSubsystem, ()->firstHighChamberRight.getX()+subX1, ()->firstHighChamberRight.getY(), firstHighChamberRight.getRotation(), 5, 5).withTimeout(1500)
//                                new DriveToPointCommand(driveSubsystem, firstHighChamberRight,5, 5).withTimeout(1500)
                        .alongWith(new WaitCommand(300).andThen(new InstantCommand(() -> secondaryArmSubsystem.setDiffy(0, -30)))),

                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new WaitCommand(200),
                                new InstantCommand(() -> specMechSubsystem.openClaw())
//                                new WaitCommand(1000),
//                                new InstantCommand(() -> specMechSubsystem.setArm(specArmWallIntake))
                        ),
                        new SequentialCommandGroup(
                                new WaitCommand(500),//keep it long for now
                                new LimelightTeleopAimer(armSubsystem, secondaryArmSubsystem, intakeSubsystem, limelightSubsystem).withTimeout(1000),
                                new WaitCommand(1000).interruptOn(()->Math.abs(armSubsystem.getSlideError())<0.3&&driveSubsystem.getTranslationalError()<0.3),
                                new WaitCommand(100),
                                new InstantCommand(()->driveSubsystem.enablePrecisePID(false))
                        )
                ),

                new ParallelizingCycles(driveSubsystem, armSubsystem, intakeSubsystem, secondaryArmSubsystem, specMechSubsystem, limelightSubsystem),
//                new ParallelizingCycles(driveSubsystem, armSubsystem, intakeSubsystem, secondaryArmSubsystem, specMechSubsystem, limelightSubsystem),
//                new ParallelizingCycles(driveSubsystem, armSubsystem, intakeSubsystem, secondaryArmSubsystem, specMechSubsystem, limelightSubsystem),

                new InstantCommand(()->Log.i("finishParrallelizing", "yes")),


                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new WaitCommand(500),
                                new DriveToPointDoubleSupplierCommand(
                                        driveSubsystem,
                                        ()->driveSubsystem.getPos().getX(),
                                        ()->driveSubsystem.getPos().getY()-10,
                                        new Rotation2d(),
                                        5,
                                        5
                                ),
                                new InstantCommand(()->Log.i("finishDoubleSupplier", "yes")),
                                new DriveToPointCommand(driveSubsystem, rightSideLeftSpikeFlip, 12, 5).withTimeout(1250),
                                new InstantCommand(()->driveSubsystem.enablePrecisePID(true)), //so we accelerate faster
                                new InstantCommand(()-> {
                                    Log.i("AutoStartXS1", String.valueOf(driveSubsystem.getPos().getX()));
                                    Log.i("AutoStartYS1", String.valueOf(driveSubsystem.getPos().getY()));
                                    Log.i("AutoStartRS1", String.valueOf(driveSubsystem.getPos().getRotation().getDegrees()));
                                    Log.i("AutoTargetXS1", String.valueOf(driveSubsystem.getTargetPos().getX()));
                                    Log.i("AutoTargetYS1", String.valueOf(driveSubsystem.getTargetPos().getY()));
                                    Log.i("AutoTargetRS1", String.valueOf(driveSubsystem.getTargetPos().getRotation().getDegrees()));
                                }),
                                new DriveToPointCommand(driveSubsystem, rightSideLeftSpikeFlip, 2, 5).withTimeout(500)
                        ),
                        new SequentialCommandGroup(
                            new RetractAfterIntake(armSubsystem, intakeSubsystem, secondaryArmSubsystem),
                            new InstantCommand(()->armSubsystem.setArmPowerCap(0.5)),
                            new WaitCommand(600),
                            new ParallelizingDropCommand(armSubsystem, intakeSubsystem, secondaryArmSubsystem),
                            new WaitCommand(100),
                            new InstantCommand(()->armSubsystem.setArmPowerCap(1.0)),
                            new InstantCommand(()->intakeSubsystem.setDiffy(15, 0)),
                            new InstantCommand(()->intakeSubsystem.clawExtraOpen()),
                            new InstantCommand(()->intakeSubsystem.openClaw()), //so that we save time when we intake later ig
                            secondaryArmSubsystem.setPitchYawSafe(0,0),
                            new WaitCommand(50)
                        )
                ),

                new FlipSpikes(driveSubsystem, armSubsystem, intakeSubsystem, secondaryArmSubsystem),
                new AutoSpecimenCycleFast(armSubsystem, intakeSubsystem, driveSubsystem, secondaryArmSubsystem, firstWallPickUp),
                new AutoSpecimenCycleFast(armSubsystem, intakeSubsystem, driveSubsystem, secondaryArmSubsystem),
                new AutoSpecimenCycleFast(armSubsystem, intakeSubsystem, driveSubsystem, secondaryArmSubsystem),
                new AutoSpecimenCycleFast(armSubsystem, intakeSubsystem, driveSubsystem, secondaryArmSubsystem),
                new AutoSpecimenCycleFast(armSubsystem, intakeSubsystem, driveSubsystem, secondaryArmSubsystem),
                new InstantCommand(()->armSubsystem.setSlidePower(1.00)),
                new WaitCommand(300),
                new InstantCommand(()->intakeSubsystem.openClaw()),
                new InstantCommand(()->armSubsystem.setSlide(ArmSubsystem.slideRetractMin)),
                new DriveToPointCommand(driveSubsystem, new Pose2d(50, -56, Rotation2d.fromDegrees(-180)), 1, 5)
        ));

        //gamepad input
        while(!isStarted() && !isStopRequested()){
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            if(currentSubIndex==1){
                if(currentGamepad1.dpad_left && !previousGamepad1.dpad_left){
                    subX1 -= 1;
                    MathUtils.clamp(subX1, -8, 6);
                }

                if(currentGamepad1.dpad_right && !previousGamepad1.dpad_right){
                    subX1 += 1;
                    subX1 = MathUtils.clamp(subX1, -8, 6);
                }
            } else if (currentSubIndex==2) {
                if(currentGamepad1.dpad_left && !previousGamepad1.dpad_left){
                    subX2 -= 1;
                    MathUtils.clamp(subX2, -8, 6);
                }

                if(currentGamepad1.dpad_right && !previousGamepad1.dpad_right){
                    subX2 += 1;
                    subX2 = MathUtils.clamp(subX2, -8, 6);
                }
            }

            if(currentGamepad1.dpad_up && !previousGamepad1.dpad_up){
                currentSubIndex+=1;
                currentSubIndex=MathUtils.clamp(currentSubIndex, 1,2);
            }

            if(currentGamepad1.dpad_down && !previousGamepad1.dpad_down){
                currentSubIndex-=1;
                currentSubIndex=MathUtils.clamp(currentSubIndex, 1,2);
            }

            //change spec mode
            if(currentGamepad1.touchpad && !previousGamepad1.touchpad){
                teleopSpec = !teleopSpec;
            }

            telemetry.addData("currentSubIndex", currentSubIndex);

            telemetry.addData("subX1 (-8,6)", subX1);

            telemetry.addData("subX2 (-8,6)", subX2);


            //specMode
            if(teleopSpec){
                telemetry.addData("specMode", "\uD83D\uDCA7\uD83D\uDCA7true\uD83D\uDC7A\uD83D\uDC7A");
            }else{
                telemetry.addData("specMode", "\uD83D\uDC72\uD83D\uDC72false⭐⭐");
            }

            telemetry.update();
        }
    }
}
