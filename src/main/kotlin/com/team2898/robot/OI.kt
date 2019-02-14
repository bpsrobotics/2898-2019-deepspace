package com.team2898.robot

import edu.wpi.first.wpilibj.XboxController

object OI {
    val opCtl = XboxController(0)

    val A
        get() = opCtl.getRawButton(1)
    val B
        get() = opCtl.getRawButton(2)
    val X
        get() = opCtl.getRawButton(3)
}