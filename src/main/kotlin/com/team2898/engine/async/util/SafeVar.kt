package com.team2898.engine.async.util

class SafeVar<T>(var value: T) {

    fun get(): T = synchronized(this) { return value }

    fun set(new: T) = synchronized(this) {
        value = new
    }

    fun runSafe(block: () -> Unit) = synchronized(this) {
        block()
    }


}
