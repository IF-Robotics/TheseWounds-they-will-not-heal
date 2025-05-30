package org.firstinspires.ftc.teamcode.commands;

import static java.lang.Double.NaN;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subSystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.SecondaryArmSubsystem;

import java.util.function.DoubleSupplier;

public class SecondaryArmCommand extends CommandBase {
    private SecondaryArmSubsystem secondaryArmSubsystem;
    private DoubleSupplier pitchAngle;
    private double yawAngle = NaN;

    public SecondaryArmCommand(SecondaryArmSubsystem secondaryArmSubsystem, double pitchAngle) {
        this.secondaryArmSubsystem = secondaryArmSubsystem;
        this.pitchAngle = ()-> pitchAngle;

        addRequirements(secondaryArmSubsystem);
    }

    public SecondaryArmCommand(SecondaryArmSubsystem secondaryArmSubsystem, double pitchAngle, double yawAngle) {
        this.secondaryArmSubsystem = secondaryArmSubsystem;
        this.pitchAngle = ()-> pitchAngle;
        this.yawAngle = yawAngle;

        addRequirements(secondaryArmSubsystem);
    }

    public SecondaryArmCommand(SecondaryArmSubsystem secondaryArmSubsystem, DoubleSupplier pitchAngle, double yawAngle) {
        this.secondaryArmSubsystem = secondaryArmSubsystem;
        this.pitchAngle = pitchAngle;
        this.yawAngle = yawAngle;

        addRequirements(secondaryArmSubsystem);
    }

    @Override
    public void execute() {
        Log.i("bruhbruhbruhauto", String.valueOf(pitchAngle));
        secondaryArmSubsystem.setDiffyPitch(pitchAngle.getAsDouble());

        if(!Double.isNaN(yawAngle)){
            secondaryArmSubsystem.setDiffyYaw(yawAngle);
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
