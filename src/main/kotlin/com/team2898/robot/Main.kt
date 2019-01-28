package com.team2898.robot

import edu.wpi.first.wpilibj.RobotBase
import java.util.function.Supplier

object Main {
    @JvmStatic
    fun main(args: Array<String>) {
        RobotBase.startRobot<Robot>(Supplier<Robot> { Robot() })
    }
}