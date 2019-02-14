package com.team2898.robot.subsystem

import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.team2898.engine.async.AsyncLooper
import com.team2898.engine.logic.GamePeriods
import com.team2898.engine.logic.Subsystem
import com.team2898.engine.motion.TalonWrapper
import com.team2898.robot.config.ARM_LEFT_MASTER
import com.team2898.robot.config.ARM_LEFT_SLAVE
import com.team2898.robot.config.ARM_RIGHT_MASTER
import com.team2898.robot.config.ARM_RIGHT_SLAVE
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard

object ArmBackup: Subsystem(50.0, "arm") {
    override val enableTimes: List<GamePeriods> = listOf(GamePeriods.AUTO, GamePeriods.TELEOP)

    val leftMaster = TalonWrapper(ARM_LEFT_MASTER)
    val leftSlace = TalonWrapper(ARM_LEFT_SLAVE)
    val rightMaster = TalonWrapper(ARM_RIGHT_MASTER)
    val rightSlave = TalonWrapper(ARM_RIGHT_SLAVE)

    var brake: Boolean = false

    init {
        leftSlace slaveTo leftMaster
        rightMaster slaveTo leftMaster
        rightSlave slaveTo leftMaster

        leftMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10)

        listOf(leftMaster, leftSlace, rightSlave, rightMaster).forEach {
            it.apply {
                setPID(0.02, 0.01, 0.001)
                configMotionAcceleration(20)
                configMotionCruiseVelocity(30)
                configVoltageCompSaturation(12.0)
            }
        }

        AsyncLooper(50.0) {
            SmartDashboard.putNumber("talon vel", leftMaster.sensorCollection.quadratureVelocity.toDouble())
            SmartDashboard.putNumber("talon pos", leftMaster.sensorCollection.quadraturePosition.toDouble())
        }.start()
    }
}
