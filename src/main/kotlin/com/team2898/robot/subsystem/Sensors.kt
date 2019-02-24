package com.team2898.robot.subsystem

import edu.wpi.first.wpilibj.DigitalInput

object Sensors {
    val hatchLimitSwitch get() = DigitalInput(6).get()
    val cargo get() = Intake.isCargoIn
}