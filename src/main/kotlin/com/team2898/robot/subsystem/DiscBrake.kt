package com.team2898.robot.subsystem

import com.team2898.engine.logic.GamePeriods
import com.team2898.engine.logic.Subsystem
import com.team2898.robot.config.BRAKE
import com.team2898.robot.config.DoubleSolenoid
import edu.wpi.first.wpilibj.DoubleSolenoid


object DiscBrake : Subsystem(50.0, "disc brake") {

    override val enableTimes: List<GamePeriods> = listOf(GamePeriods.AUTO, GamePeriods.TELEOP)

    private val brakeSelenoid = DoubleSolenoid(BRAKE)

    fun brakeUpdate(b: Boolean = Arm.brake) {
        if (b) brakeSelenoid.set(DoubleSolenoid.Value.kForward)
        else brakeSelenoid.set(DoubleSolenoid.Value.kReverse)
    }
}
