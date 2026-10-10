import unittest
import metrics

class MetricsTest(unittest.TestCase):
    def test_intervals_keep_zero_and_reversal(self):
        x=metrics.timing([0,10,10,9,20],1)
        self.assertEqual(x['zero'],1)
        self.assertEqual(x['backwards'],1)
        self.assertEqual(x['min_ms'],-1)
    def test_percentile_linear_and_short(self):
        self.assertEqual(metrics.percentile([0,10],.95),9.5)
        self.assertIsNone(metrics.percentile([],.5))

if __name__=='__main__': unittest.main()
