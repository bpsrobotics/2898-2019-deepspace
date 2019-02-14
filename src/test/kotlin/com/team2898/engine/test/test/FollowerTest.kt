package com.team2898.engine.test.test

import com.team254.lib.util.motion.MotionProfileConstraints
import com.team254.lib.util.motion.MotionProfileGenerator
import com.team254.lib.util.motion.MotionProfileGoal
import com.team254.lib.util.motion.MotionState
import com.team2898.engine.math.linear.Matrix
import com.team2898.engine.math.linear.T
import com.team2898.engine.math.linear.get
import com.team2898.engine.math.linear.row
import com.team2898.robot.subsystem.Arm
import io.kotlintest.specs.StringSpec
import java.io.File


class FollowerTest: StringSpec() {
    init {
        val sb = StringBuilder()
        val const = MotionProfileConstraints(3.0, 1.0)
        val prof = MotionProfileGenerator.generateProfile(const, MotionProfileGoal(5.0), MotionState(0.0, 0.0,0.0,0.0))
        println(prof)

        var i = 0.0
        while (i < prof.endTime() + 0.5) {
            i += 0.05
            val state = prof.stateByTimeClamped(i)
            val r = Matrix(Matrix(arrayOf(row(state.pos(), state.vel()))).T.data)
            val u = Arm.genU(r)
            sb.append("$i, ${Arm.xHat[0, 0]}, ${Arm.xHat[1, 0]}, ${state.pos()}, ${state.vel()}, ${u[0, 0]}\n")
        }
        File("test.csv").writeText(sb.toString())
    }
}

