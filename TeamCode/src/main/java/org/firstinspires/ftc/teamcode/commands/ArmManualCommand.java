package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;

import java.util.function.DoubleSupplier;

import static org.firstinspires.ftc.teamcode.other.Globals.*;

public class ArmManualCommand extends CommandBase {
    private ArmSubsystem armSubsystem;
    private DoubleSupplier armPower = null, slidePower = null;

    public ArmManualCommand(ArmSubsystem armSubsystem, DoubleSupplier armPower, DoubleSupplier slidePower) {
        this.armSubsystem = armSubsystem;
        this.armPower = armPower;
        this.slidePower = slidePower;

        addRequirements(armSubsystem);
    }

    public ArmManualCommand(ArmSubsystem armSubsystem, DoubleSupplier slidePower) {
        this.armSubsystem = armSubsystem;
        this.armPower = null;
        this.slidePower = slidePower;

        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        manualSlides = true;
        if(!(armPower == null)){
            manualArm = true;
        }
    }

    @Override
    public void execute() {
        if(armPower != null){
            armSubsystem.manualArm(armPower.getAsDouble(), -slidePower.getAsDouble());
        } else {
            armSubsystem.manualArm(0, -slidePower.getAsDouble());
        }

    }

    @Override
    public void end(boolean interrupted){
        manualArm = false;
        manualSlides = false;

        if(armPower != null){
            armSubsystem.setArm(armSubsystem.getArmAngle());
        }

        armSubsystem.setSlide(armSubsystem.getSlideExtention());
        armSubsystem.setArmCoordinates(armSubsystem.getCurrentX(), armSubsystem.getCurrentY());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}

