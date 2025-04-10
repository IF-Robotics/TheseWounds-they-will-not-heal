package org.firstinspires.ftc.teamcode.commandGroups;

import static org.firstinspires.ftc.teamcode.other.Globals.*;
import static org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem.*;


import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.commands.ArmCoordinatesCommand;
import org.firstinspires.ftc.teamcode.commands.ArmManualCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.WaitForArmCommand;
import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSubsystem;

public class ClimbLevel3 extends SequentialCommandGroup {

    public ClimbLevel3(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, IMU gyro){
        addCommands(

                //engage hooks intially
                new InstantCommand(() -> armSubsystem.setSlide(armSubsystem.slideWristOffset)),

                //wait
                new WaitCommand(500),

                //engage the pto
                new InstantCommand()




//                //move endstop out of the way
//                //move intake out of the way
//                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, pitchFrontHighChamber, rollFrontHighChamber),
//
//                //Climb to first rung
//                new ArmManualCommand(armSubsystem, () -> 1, () -> -1).withTimeout(2000),
//                //Climb to second rung
//                new InstantCommand(() -> armSubsystem.setArmP(climbingArmP)),
//                //rotate arm up to second rung
//                new InstantCommand(() -> armSubsystem.setSlide(8)),
//                new WaitForArmCommand(armSubsystem, 70, 2, .4),
////                new ArmManualCommand(armSubsystem, () -> -0.6, () -> 0).interruptOn(() -> gyro.getRobotYawPitchRollAngles().getPitch() > 90).withTimeout(2000),
////                new InstantCommand(() -> armSubsystem.setEndstop(ArmSubsystem.Endstop.DOWN)),
////                new InstantCommand(() -> armSubsystem.setEndstop(ArmSubsystem.Endstop.DOWN)),
//                new WaitCommand(200),
//                //correct with imu
//                new RunCommand(()-> armSubsystem.setArm(85-gyro.getRobotYawPitchRollAngles().getPitch())).withTimeout(2000),
//                //extend slides with imu correction
//                new ParallelDeadlineGroup(
//                    new WaitCommand(600),
//                    new InstantCommand(() -> armSubsystem.setSlide(24.5)),
//                    //correcting with imu
//                    new RunCommand(()-> armSubsystem.setArm(85-gyro.getRobotYawPitchRollAngles().getPitch()))
//                ),
//                //rotate arm to second rung with imu
//                new RunCommand(()-> armSubsystem.setArm(60-gyro.getRobotYawPitchRollAngles().getPitch())).withTimeout(1000),
//                new InstantCommand(() -> armSubsystem.setArmP(kParm)),
//                //Move arm back to rotate the robot down while retracting linear slides until first rung is at the end of the robot ramp
//                new InstantCommand(() -> armSubsystem.setSlide(0)),
//                new InstantCommand(() -> armSubsystem.setArm(132)),
//                //wait
//                new WaitCommand(1000),
//                //Retract linear slides completely
//                new InstantCommand(() -> armSubsystem.setArmP(.1)),
//                new InstantCommand(() -> armSubsystem.setSlideP(.1)),
//                new ArmManualCommand(armSubsystem, () -> 1, () -> -1).withTimeout(800),
//                new ArmManualCommand(armSubsystem, () -> 1, () -> -.5).withTimeout(2000),
//                new ArmCoordinatesCommand(armSubsystem, armFoldX, armFoldY)
        );
        addRequirements(armSubsystem, intakeSubsystem);
    }
}
