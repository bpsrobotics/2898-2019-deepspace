package com.team2898.robot

import com.team2898.engine.async.AsyncLooper
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import java.lang.Math.max
import java.lang.Math.pow
import kotlin.math.abs
import kotlin.math.sign

object OI {

    init {
        AsyncLooper(50.0) {
            SmartDashboard.putNumber("pov", opCtl.povCount.toDouble())
            SmartDashboard.putNumber("pov get", opCtl.pov.toDouble())
        }.start()
    }

    fun deadzone(value: Double): Double {
        if (abs(value) < 0.10) return 0.0
        return value
    }

    fun cube(value: Double): Double = pow(value, 3.0)
    fun square(value: Double): Double = pow(value, 2.0) * sign(value)

    fun process(value: Double, deadzone: Boolean = true, cube: Boolean = false, square: Boolean = false): Double {
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

//        val opCtl = XboxController(0)

    val throttle
        get() = process(driverController.getRawAxis(1), square = true)
    val turn
        get() = process(driverController.getRawAxis(4), square = true)
    val quickTurn: Boolean
        get() = process(max(driverController.getRawAxis(2), driverController.getRawAxis(3))) != 0.0
    val leftTrigger
        get() = process(driverController.getRawAxis(2) * 0.8, square = true)
    val rightTrigger
        get() = process(driverController.getRawAxis(3) * 0.8, square = true)
    val drLShoulder
        get() = driverController.getRawButton(5)
    val drRShoulder
        get() = driverController.getRawButton(6)

    val drA
        get() = driverController.getRawButton(1)
    val drB
        get() = driverController.getRawButton(2)
    val drX
        get() = driverController.getRawButton(3)




    val opCtl = Joystick(1)
    val A
        get() = opCtl.getRawButton(1)
    val B
        get() = opCtl.getRawButton(2)
    val X
        get() = opCtl.getRawButton(3)

}