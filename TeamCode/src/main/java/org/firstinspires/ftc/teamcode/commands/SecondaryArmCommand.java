package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subSystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.SecondaryArmSubsystem;

public class SecondaryArmCommand extends CommandBase {
    private SecondaryArmSubsystem secondaryArmSubsystem;
    private double pitchAngle;
    private double rollAngle;

    public SecondaryArmCommand(SecondaryArmSubsystem secondaryArmSubsystem, double pitchAngle, double rollAngle) {
        this.secondaryArmSubsystem = secondaryArmSubsystem;
        this.pitchAngle = pitchAngle;
        this.rollAngle = rollAngle;

        addRequirements(secondaryArmSubsystem);
    }

    @Override
    public void execute() {


        secondaryArmSubsystem.setDiffy(pitchAngle, rollAngle);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
