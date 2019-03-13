package com.team2898.robot.commands.testbot

import com.team254.lib.util.motion.MotionProfileConstraints
import com.team254.lib.util.motion.MotionProfileGenerator
import com.team254.lib.util.motion.MotionProfileGoal
import com.team254.lib.util.motion.MotionState
import com.team2898.engine.async.NotifierLooper
import com.team2898.engine.math.linear.Matrix
import com.team2898.engine.math.linear.row
import com.team2898.engine.subsystems.Navx
import com.team2898.robot.subsystem.Drivetrain
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.command.Command

object TestAuto: Command() {
    val looper: NotifierLooper
    var prof = MotionProfileGenerator.generateProfile(MotionProfileConstraints(3.0, 2.0),
            MotionProfileGoal(0.0),
            MotionState(0.0, 0.0, 0.0, 0.0)
    )

    override fun initialize() {
        prof = MotionProfileGenerator.generateProfile(MotionProfileConstraints(3.0, 2.0),
                MotionProfileGoal(0.0),
                MotionState(Timer.getFPGATimestamp(), 0.0, 0.0, 0.0))
        Navx.reset()
        Drivetrain.encoders { reset() }
        looper.start()
    }
    init {
        val dt = 0.01
        looper = NotifierLooper(100.0) {
            val state = prof.stateByTimeClamped(Timer.getFPGATimestamp())
            val next = prof.stateByTimeClamped(Timer.getFPGATimestamp() + dt)
            val r = Matrix(arrayOf(row(state.vel()), row(state.vel())))
            val nextR = Matrix(arrayOf(row(next.vel()), row(next.vel())))
            val u = Drivetrain.genU(r, nextR, x = Drivetrain.x)
            Drivetrain.openLoopPower(u)
        }
    }

    override fun isFinished(): Boolean {
        return Timer.getFPGATimestamp() > prof.endTime()
    }

    override fun end() {
        looper.end()
    }
}
