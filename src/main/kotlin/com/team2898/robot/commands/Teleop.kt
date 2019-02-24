package com.team2898.robot.commands

import com.team2898.engine.motion.CheesyDrive
import com.team2898.robot.OI
import com.team2898.robot.config.ArmConf.*
import com.team2898.robot.subsystem.Arm
import com.team2898.robot.subsystem.DiscBrake
import com.team2898.robot.subsystem.Drivetrain
import com.team2898.robot.subsystem.Intake
import edu.wpi.first.wpilibj.command.Command
import kotlinx.serialization.ImplicitReflectionSerializer

class Teleop : Command() {

    override fun initialize() {
//        Arm.updateTarget(ARM_OFFSET)
    }

    override fun isFinished(): Boolean = false

    @ImplicitReflectionSerializer
    override fun execute() {
        Intake.hatch = OI.opCtl.getRawButton(1)
        Intake.hatchIntake()
        DiscBrake.brakeUpdate()
        if (OI.opCtl.getRawButton(2)) Arm.updateTarget(0.53)
        if (OI.opCtl.getRawButton(7)) Arm.updateTarget(hatchl3)
        if (OI.opCtl.getRawButton(9)) Arm.updateTarget(hatchl2)
        if (OI.opCtl.getRawButton(11)) Arm.updateTarget(hatchl1)
        if (OI.opCtl.getRawButton(8)) Arm.updateTarget(cargol3)
        if (OI.opCtl.getRawButton(10)) Arm.updateTarget(cargol2)
        if (OI.opCtl.getRawButton(12)) Arm.updateTarget(cargol1)
        if (OI.opCtl.getRawButton(5)) Arm.updateTarget(cargo)
        if (OI.driverController.aButton) Arm.updateTarget(0.0)
        if (OI.driverController.bButton || OI.opCtl.pov == 0) Arm.updateTarget(Arm.currentPos.pos() + 0.1)
        if (OI.driverController.xButton || OI.opCtl.pov == 180) Arm.updateTarget(Arm.currentPos.pos() - 0.1)
        if (OI.opCtl.getRawButton(2)) Intake.cargoReset()

        Drivetrain.openLoopPower(CheesyDrive.updateCheesy(
                (if (!OI.quickTurn) OI.turn else -OI.leftTrigger + OI.rightTrigger),
                -OI.throttle,
                OI.quickTurn,
                false
        ))
    }
}
