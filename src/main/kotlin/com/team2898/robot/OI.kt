package com.team2898.robot

import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.XboxController
import kotlin.math.abs
import kotlin.math.pow
import kotlin.math.sign

object OI {

    private fun deadzone(value: Double): Double {
        if (abs(value) < 0.10) return 0.0
        return value
    }

    private fun cube(value: Double): Double = value.pow(3.0)
    private fun square(value: Double): Double = value.pow(2.0) * sign(value)

    private fun process(value: Double, deadzone: Boolean = true, cube: Boolean = false, square: Boolean = false): Double {
        var localValue = value
        if (deadzone)
            localValue = deadzone(value)
        if (cube) {
            localValue = cube(localValue)
        } else if (square) {
            localValue = square(localValue)
        }

        return localValue
    }

    val driverController = XboxController(0)

    val throttle get() = process(driverController.getRawAxis(1), square = true)
    val turn get() = process(driverController.getRawAxis(4), square = true)
    val quickTurn: Boolean get() = process(driverController.getRawAxis(2).coerceAtLeast(driverController.getRawAxis(3))) != 0.0
    val leftTrigger get() = process(driverController.getRawAxis(2) * 0.8, square = true)
    val rightTrigger get() = process(driverController.getRawAxis(3) * 0.8, square = true)

    val opCtl = Joystick(1)
    val A get() = opCtl.getRawButton(1)

}