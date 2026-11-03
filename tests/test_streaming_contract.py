import unittest
import numpy as np
from piauto.adapters.streaming import SampleBatch, validate_batch

class StreamingContractTests(unittest.TestCase):
    def test_valid_batch_quality(self):
        b=SampleBatch("dev","m1","t1",np.arange(100)/1000,
                      {"position":np.zeros(100)},{"position":"rad"},1,"device_clock",True)
        q=validate_batch(b,["position"])
        self.assertEqual(q["samples"],100)
        self.assertEqual(q["timestamp_source"],"device_clock")
    def test_missing_signal_rejected(self):
        b=SampleBatch("d","m","t",np.arange(10)/100,{"position":np.zeros(10)})
        with self.assertRaises(ValueError):validate_batch(b,["velocity"])
    def test_nonmonotonic_clock_rejected(self):
        b=SampleBatch("d","m","t",np.array([0.,.1,.1]),{"position":np.zeros(3)})
        with self.assertRaises(ValueError):b.validate()
    def test_high_drop_rate_rejected(self):
        b=SampleBatch("d","m","t",np.arange(10)/100,{"position":np.zeros(10)},dropped_frames=5)
        with self.assertRaises(ValueError):validate_batch(b,["position"])
