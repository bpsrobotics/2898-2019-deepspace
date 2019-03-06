package com.team2898.robot.commands.testbot

import com.team2898.engine.math.linear.Matrix
import com.team2898.engine.math.linear.T
import com.team2898.engine.math.linear.row
import com.team2898.engine.motion.CheesyDrive
import com.team2898.robot.OI
import com.team2898.robot.OI.leftTrigger
import com.team2898.robot.OI.quickTurn
import com.team2898.robot.OI.rightTrigger
import com.team2898.robot.OI.throttle
import com.team2898.robot.OI.turn
import com.team2898.robot.subsystem.Drivetrain
import edu.wpi.first.wpilibj.command.Command
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard

class TestTeleop: Command() {
    override fun isFinished(): Boolean  =false

    override fun initialize() {
        Drivetrain.encoders {
            reset()
        }
    }

    override fun execute() {
        Drivetrain.openLoopPower(
                CheesyDrive.updateCheesy(
                        (if (!quickTurn) turn else -leftTrigger + rightTrigger),
                        -throttle, // Remove "-" if robot goes backwards
                        quickTurn,
                        true
                )
        )
    }
}