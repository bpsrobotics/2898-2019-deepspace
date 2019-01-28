package com.team2898.robot

import com.team2898.engine.async.AsyncLooper
import com.team2898.robot.subsystem.Drivetrain
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard

class Robot: TimedRobot() {
    override fun finalize() {
        AsyncLooper(20.0) {
            SmartDashboard.putNumber("left", Drivetrain.leftEnc.get().toDouble())
            SmartDashboard.putNumber("right", Drivetrain.rightEnc.get().toDouble())
        }.start()
    }

}