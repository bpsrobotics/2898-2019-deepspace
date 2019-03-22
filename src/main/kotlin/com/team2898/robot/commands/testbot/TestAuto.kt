package com.team2898.robot.commands.testbot

import com.team254.lib.util.motion.MotionProfileConstraints
import com.team254.lib.util.motion.MotionProfileGenerator
import com.team254.lib.util.motion.MotionProfileGoal
import com.team254.lib.util.motion.MotionState
import com.team2898.engine.async.NotifierLooper
import com.team2898.engine.math.linear.Matrix
import com.team2898.engine.math.linear.row
import com.team2898.engine.motion.CheesyDrive
import com.team2898.engine.motion.DriveSignal
import com.team2898.engine.subsystems.Navx
import com.team2898.robot.OI
import com.team2898.robot.subsystem.Drivetrain
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.command.Command

object TestAuto: Command() {
    const val Kp = 0.6
    const val Ki = 0.2
    const val Kd = 0.1

    override fun isFinished(): Boolean {
        return false
    }

    override fun execute() {
        var power = CheesyDrive.updateCheesy(
                (if (!OI.quickTurn) OI.turn else -OI.leftTrigger + OI.rightTrigger),
                -OI.throttle,
                OI.quickTurn,
                false
        )
        if (OI.driverController.pov == 90) {
            power = DriveSignal(left = 0.2, right = -0.1)
        }
        if (OI.driverController.pov == 270) {
            power = DriveSignal(left = -0.1, right = 0.2)
        }
        if (OI.driverController.pov == 0 || OI.driverController.pov == 360) {
            power = DriveSignal(0.2, 0.2)
        }
        if (OI.driverController.pov == 180) {
            power = DriveSignal(-0.2, -0.2)
        }

        if (OI.driverController.getRawButton(5)) {

        }
        Drivetrain.openLoopPower(power)
    }

}
