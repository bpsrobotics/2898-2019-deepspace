package com.team2898.robot.commands

import com.team2898.engine.motion.CheesyDrive
import com.team2898.engine.motion.DriveSignal
import com.team2898.robot.OI
import com.team2898.robot.config.ArmConf.*
import com.team2898.robot.subsystem.Arm
import com.team2898.robot.subsystem.DiscBrake
import com.team2898.robot.subsystem.Drivetrain
import edu.wpi.first.wpilibj.command.Command

object Teleop : Command() {

    override fun initialize() {
        Arm.controlLoop.start()
    }

    override fun isFinished(): Boolean = false

    override fun execute() {
        DiscBrake.brakeUpdate()
        if (OI.opCtl.getRawButton(7)) Arm.updateTarget(hatchl3 + 0.25)
        if (OI.opCtl.getRawButton(9)) Arm.updateTarget(hatchl2 + 0.45)
        if (OI.opCtl.getRawButton(11)) Arm.updateTarget(hatchl1 + 0.45)
        if (OI.opCtl.getRawButton(8)) Arm.updateTarget(cargol3)
        if (OI.opCtl.getRawButton(10)) Arm.updateTarget(cargol2 + 0.1)
        if (OI.opCtl.getRawButton(12)) Arm.updateTarget(cargol1)
        if (OI.opCtl.getRawButton(5)) Arm.updateTarget(cargo)
        if (OI.driverController.aButton) Arm.updateTarget(0.0)
        if (OI.driverController.bButton || OI.opCtl.pov == 0) Arm.updateTarget(Arm.currentPos.pos() + 0.1)
        if (OI.driverController.xButton || OI.opCtl.pov == 180) Arm.updateTarget(Arm.currentPos.pos() - 0.1)

        var power = CheesyDrive.updateCheesy(
                (if (!OI.quickTurn) OI.turn else -OI.leftTrigger + OI.rightTrigger),
                -OI.throttle,
                OI.quickTurn,
                false
        )
        if (OI.driverController.getRawButton(6)){
            power = power.adjPower(0.5)
        }
        if (OI.driverController.pov == 90) {
            power = DriveSignal(left = 0.2, right = -0.1)
        }
        if (OI.driverController.pov == 270) {
            power = DriveSignal(left = -0.1, right = 0.2)
        }
        if (OI.driverController.pov == 0 || OI.driverController.pov == 360) {
            power = DriveSignal(0.3, 0.3)
        }
        if (OI.driverController.pov == 180) {
            power = DriveSignal(-0.3, -0.3)
        }
        Drivetrain.openLoopPower(power)
    }

    fun DriveSignal.adjPower(power: Double): DriveSignal {
        return DriveSignal(this.left * power, this.right * power, this.brake)
    }
}
