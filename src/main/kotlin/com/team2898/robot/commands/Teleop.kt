package com.team2898.robot.commands

import com.team2898.engine.motion.CheesyDrive
import com.team2898.engine.motion.DriveSignal
import com.team2898.robot.OI
import com.team2898.robot.subsystem.Arm
import com.team2898.robot.subsystem.DiscBrake
import com.team2898.robot.subsystem.Drivetrain
import edu.wpi.first.wpilibj.command.Command

object Teleop : Command() {

//    override fun initialize() {
//        Arm.controlLoop.start()
//    }

    override fun isFinished(): Boolean = false

    override fun execute() {
        DiscBrake.brakeUpdate()
        // TODO: move to OI
        if (OI.hatchR3) Arm.updatePosition(Arm.ArmPositions.HATCH_R3)
        if (OI.hatchR2) Arm.updatePosition(Arm.ArmPositions.HATCH_R2)
        if (OI.hatchR1) Arm.updatePosition(Arm.ArmPositions.HATCH_R1)
        if (OI.cargoR3) Arm.updatePosition(Arm.ArmPositions.CARGO_R3)
        if (OI.cargoR2) Arm.updatePosition(Arm.ArmPositions.CARGO_R2)
        if (OI.cargoR1) Arm.updatePosition(Arm.ArmPositions.CARGO_R1)
        if (OI.cargoC) Arm.updatePosition(Arm.ArmPositions.CARGO_C)
        if (OI.driverController.aButton) Arm.updatePosition(Arm.ArmPositions.BOTTOM)
//        if (OI.driverController.bButton || OI.opCtl.pov == 0) Arm.updateTarget(Arm.currentPos.pos() + 0.1)
//        if (OI.driverController.xButton || OI.opCtl.pov == 180) Arm.updateTarget(Arm.currentPos.pos() - 0.1)

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

    private fun DriveSignal.adjPower(power: Double): DriveSignal {
        return DriveSignal(this.left * power, this.right * power, this.brake)
    }
}
