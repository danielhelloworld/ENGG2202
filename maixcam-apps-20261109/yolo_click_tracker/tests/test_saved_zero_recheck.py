"""Saved-zero feedback rebuild on an eccentric disabled load; model only."""
import unittest
import test_tracker as app
import test_separate_controls as controls

m=app.m


class SavedZeroRecheckTests(unittest.TestCase):
    setUp=controls.SeparateControlTests.setUp
    make_driver=controls.SeparateControlTests.make_driver

    def test_recheck_mailbox_does_not_read_or_write_uart(self):
        d,bus=self.make_driver((0.,108.))
        before=len(bus.writes);d.request_recheck()
        self.assertEqual(len(bus.writes),before)
        self.assertEqual(d.phase,'RECHECK_PENDING')

    def test_falling_disabled_load_with_nonzero_speed_can_rebuild_saved_reference(self):
        d,bus=self.make_driver((0.,108.))
        original=bus.write
        def falling(packet):
            if packet[2]==m.FUNCTION_FEEDBACK and packet[3]==m.FEEDBACK_MECHANICAL_ANGLE:
                bus.physical[2]+=.8
            return original(packet)
        bus.write=falling;bus.nonzero_speed=True
        before=len(bus.writes);d.request_recheck();d._perform_recheck()
        writes=bus.writes[before:]
        self.assertEqual(d.recheck_status,'PASSED')
        self.assertTrue(d.ready and d.reference_valid and d.zero_confirmed)
        self.assertTrue(d.snapshot()['outside_work_limits'])
        self.assertFalse(d.armed or d.arm_requested or d.home_requested)
        self.assertFalse(any(p[2] in (3,6,8,9,10) for p in writes))
        self.assertEqual([(p[1],p[3]) for p in writes if p[2]==m.FUNCTION_FEEDBACK],[(1,2),(2,2)])

    def test_missing_axis_never_passes_using_old_cached_feedback(self):
        d,bus=self.make_driver((0.,0.));bus.drop_feedback={2}
        d.request_recheck()
        with self.assertRaises(m.FeedbackUnavailable):d._perform_recheck()
        self.assertEqual(d.recheck_status,'FAILED');self.assertEqual(d.phase,'FAULT')
        self.assertFalse(d.armed or d.reference_valid or d.ready or d.zero_confirmed)
        self.assertFalse(any(bus.enabled.values()))
        self.assertIsNotNone(d.recheck_finished)

    def test_recheck_archives_prior_fault_without_rearming_or_hiding_it(self):
        d,bus=self.make_driver((0.,0.))
        d.fault='original fault';d.phase='FAULT'
        d.last_feedback_fault=dict(reason='original feedback failure')
        d.request_recheck();d._perform_recheck()
        previous=d.snapshot()['recheck_diagnostic']['previous']
        self.assertEqual(previous['reason'],'original fault')
        self.assertEqual(previous['feedback_fault']['reason'],'original feedback failure')
        self.assertIsNone(d.fault);self.assertFalse(d.arm_requested)
