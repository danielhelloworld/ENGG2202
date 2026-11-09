"""Retarget unchanged, applicable regression tests to the new standalone module."""
import importlib
import json
from pathlib import Path
import sys
import unittest

ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT.parent/'tests'))
import test_wide_motion_v42 as wide

MODULES=('test_speed_tracking','test_feedback_response','test_feedback_scheduling','test_continuous_yaw',
         'test_arming_guard','test_fixed_hold','test_manual_fault_regression','test_saved_zero_recheck',
         'test_startup_recovery','test_uart_frame_gap','test_angle_move','test_fast_manual','test_fast_return')


def suite(include_http=False):
    modules=[importlib.import_module(name) for name in MODULES]
    if include_http:modules.append(importlib.import_module('test_tracker'))
    # Imported regression helpers share module-level aliases. Rebind only the
    # old click-tracker bundle; protocol fixtures keep their independent plant.
    for module in list(sys.modules.values()):
        if getattr(module,'m',None) is wide.old:module.m=wide.m
    result=unittest.TestSuite(unittest.defaultTestLoader.loadTestsFromModule(module) for module in modules
                              if module.__name__!='test_tracker')
    if include_http:result.addTests(unittest.defaultTestLoader.loadTestsFromTestCase(sys.modules['test_tracker'].HTTPTests))
    return result


if __name__=='__main__':
    include_http='--http' in sys.argv
    result=unittest.TextTestRunner(verbosity=1).run(suite(include_http))
    (ROOT/('compat_http_results.json' if include_http else 'compat_results.json')).write_text(json.dumps(dict(
        tests=result.testsRun,failures=len(result.failures),errors=len(result.errors),passed=result.wasSuccessful()),indent=2)+'\n',encoding='utf-8')
    raise SystemExit(not result.wasSuccessful())
