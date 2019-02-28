package com.team2898.engine.async

import edu.wpi.first.wpilibj.Notifier


class NotifierLooper(
        val hz: Double,
        val block: () -> Unit
) {
    private fun init(): Runnable = Runnable(block)

    private val job = Notifier(init())

    fun start() {
        job.startPeriodic(1/hz)
    }

    fun end() {
        job.stop()
    }

}

