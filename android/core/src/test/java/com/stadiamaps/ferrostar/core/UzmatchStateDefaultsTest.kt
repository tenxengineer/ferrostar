package com.stadiamaps.ferrostar.core

import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test
import uniffi.ferrostar.TemporalMatchState

class UzmatchStateDefaultsTest {

  @Test
  fun temporalMatchStateHasEmptyFfiDefaults() {
    val state = TemporalMatchState()

    assertTrue(state.candidates.isEmpty())
    assertNull(state.previousLocation)
  }
}
