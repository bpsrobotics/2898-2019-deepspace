package com.team2898.robot.commands.testbot

import com.team2898.engine.math.linear.Matrix
import com.team2898.engine.math.linear.T
import com.team2898.engine.math.linear.row
import com.team2898.robot.OI
import com.team2898.robot.subsystem.Drivetrain
import edu.wpi.first.wpilibj.command.Command

class TestTeleop: Command() {
    override fun isFinished(): Boolean  =false

    override fun initialize() {
        Drivetrain.encoders {
            reset()
        }
    }

    override fun execute() {
        val linear = OI.throttle * 10
        val ang = OI.turn * 3
        val left = (-(ang * 1.9) + (2 * linear)) / 2
        val right = ((ang * 1.9) + (2 * linear)) / 2
        Drivetrain.r = Matrix(row(left, right)).T
    }
}