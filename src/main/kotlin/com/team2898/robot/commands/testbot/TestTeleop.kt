package com.team2898.robot.commands.testbot

import com.team2898.engine.math.linear.Matrix
import com.team2898.engine.math.linear.T
import com.team2898.engine.math.linear.row
import com.team2898.robot.OI
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
        val ang = OI.turn * 3
        val linear = OI.throttle * 10 - ang * 2
        val left = (-(ang * 1.9) + (2 * linear)) / 2
        val right = ((ang * 1.9) + (2 * linear)) / 2
        Drivetrain.r = Matrix(arrayOf(row(left), row(right)))
//        SmartDashboard.putNumber("left", left)
//        SmartDashboard.putNumber("right", right)
    }
}