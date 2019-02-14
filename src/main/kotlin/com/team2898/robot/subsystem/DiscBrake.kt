package com.team2898.robot.subsystem

import com.team2898.engine.async.AsyncLooper
import com.team2898.engine.logic.GamePeriods
import com.team2898.engine.logic.Subsystem
import com.team2898.robot.OI
import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser


object DiscBrake : Subsystem(50.0, "disc brake") {

    override val enableTimes: List<GamePeriods> = listOf(GamePeriods.AUTO, GamePeriods.TELEOP)

    val brakeSelenoid = DoubleSolenoid(7, 2)


    fun brakeUpdate() {
        if (Arm.brake) brakeSelenoid.set(DoubleSolenoid.Value.kForward)
        else brakeSelenoid.set(DoubleSolenoid.Value.kReverse)
    }
}
