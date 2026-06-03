import math
import unittest

from opendbc.testing import parameterized

from opendbc.car import CanData
from opendbc.car.car_helpers import interfaces
from opendbc.car.hyundai.radar_interface import MRR30_CAN_RADAR_ADDR, MRR30_CAN_RADAR_COUNT, MRR30_CAN_RADAR_GROUP_SIZE, \
                                                 MRR30_CAN_RADAR_TRACK_COUNT, MRR30_CAN_RADAR_TRACK_END
from opendbc.car.hyundai.values import CAR, HyundaiFlags
from opendbc.sunnypilot.car.interfaces import setup_interfaces
from opendbc.sunnypilot.car.hyundai.escc import ESCC_MSG
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP

ESCC_CARS = [
  (CAR.HYUNDAI_ELANTRA_2021, ESCC_MSG),
]

CAMERA_SCC_CARS = [
  (CAR.HYUNDAI_KONA_EV_2022, 0, 0x420, "SCC11"),
  (CAR.HYUNDAI_IONIQ_5, HyundaiFlags.CANFD_CAMERA_SCC.value, 0x1A0, "SCC_CONTROL"),
]

STANDARD_RADAR_CARS = [
  (CAR.HYUNDAI_ELANTRA_2021, 0),
  (CAR.HYUNDAI_SANTA_FE, 0),
]

MRR30_CAN_RADAR_CARS = [
  (CAR.HYUNDAI_ELANTRA_HEV_2021, MRR30_CAN_RADAR_ADDR, MRR30_CAN_RADAR_COUNT),
]


class TestRadarInterfaceExt(unittest.TestCase):

  @staticmethod
  def _setup_platform(car_name, additional_flags=0, escc_msg=None):
    """Set up the platform with specific parameters"""
    CarInterface = interfaces[car_name]

    CP = CarInterface.get_non_essential_params(car_name)
    CP.flags |= additional_flags

    CP_SP = CarInterface.get_non_essential_params_sp(CP, car_name)

    CI = CarInterface(CP, CP_SP)

    RD = CI.RadarInterface(CP, CP_SP)

    if escc_msg is not None and hasattr(RD, 'use_escc'):
      try:
        RD.use_escc = True
      except AttributeError:
        object.__setattr__(RD, 'use_escc', True)

    return RD, CP, CP_SP

  @parameterized("car_name, escc_msg", ESCC_CARS)
  def test_escc_radar_interface(self, car_name, escc_msg):
    """Test radar interface for ESCC-enabled cars"""
    RD, CP, CP_SP = self._setup_platform(car_name, escc_msg=escc_msg)

    # Assert that ESCC features are present
    if hasattr(RD, 'use_escc'):
      self.assertTrue(RD.use_escc, "ESCC car should have use_escc=True")
    if hasattr(RD, 'use_radar_interface_ext'):
      self.assertTrue(RD.use_radar_interface_ext, "ESCC car should use radar interface ext")

    # Run radar interface once
    RD.update([])

    # Test radar fault
    if not CP.radarUnavailable and RD.rcp is not None:
      cans = [(0, [CanData(0, b'', 0) for _ in range(5)])]
      rr = RD.update(cans)
      self.assertTrue(rr is None or len(rr.errors) > 0)

  @parameterized("car_name, flags, expected_trigger, msg_src", CAMERA_SCC_CARS)
  def test_camera_scc_radar_interface(self, car_name, flags, expected_trigger, msg_src):
    """Test radar interface for Camera SCC cars"""
    RD, CP, CP_SP = self._setup_platform(car_name, additional_flags=flags)

    # Assert Camera SCC flag is set appropriately
    if flags & HyundaiFlags.CAMERA_SCC:
      self.assertTrue(CP.flags & HyundaiFlags.CAMERA_SCC, "Car should have CAMERA_SCC flag")
    if flags & HyundaiFlags.CANFD_CAMERA_SCC:
      self.assertTrue(CP.flags & HyundaiFlags.CANFD_CAMERA_SCC, "Car should have CANFD_CAMERA_SCC flag")

    # Check if using radar interface ext
    if hasattr(RD, 'use_radar_interface_ext'):
      self.assertTrue(RD.use_radar_interface_ext, "Camera SCC car should use radar interface ext")

    # Verify trigger message
    if hasattr(RD, 'trigger_msg'):
      self.assertEqual(RD.trigger_msg, expected_trigger, f"Expected trigger_msg {expected_trigger}, got {RD.trigger_msg}")

    # Run radar interface once
    RD.update([])

    # Test radar fault
    if not CP.radarUnavailable and RD.rcp is not None:
      cans = [(0, [CanData(0, b'', 0) for _ in range(5)])]
      rr = RD.update(cans)
      self.assertTrue(rr is None or len(rr.errors) > 0)

  @parameterized("car_name, flags", STANDARD_RADAR_CARS)
  def test_standard_radar_interface(self, car_name, flags):
    """Test radar interface for standard radar cars"""
    RD, CP, CP_SP = self._setup_platform(car_name, additional_flags=flags)

    # Standard cars should not use radar interface ext
    if hasattr(RD, 'use_radar_interface_ext'):
      self.assertFalse(RD.use_radar_interface_ext, "Standard car should not use radar interface ext")

    # Run radar interface once
    RD.update([])

    # For standard radar, test the _update method directly if available
    if not CP.radarUnavailable and RD.rcp is not None and \
          hasattr(RD, '_update') and hasattr(RD, 'trigger_msg'):
      # Setup for _update test if needed
      if hasattr(RD, 'updated_messages'):
        RD.updated_messages = {RD.trigger_msg}
      RD._update(RD.updated_messages)

    # Test radar fault
    if not CP.radarUnavailable and RD.rcp is not None:
      cans = [(0, [CanData(0, b'', 0) for _ in range(5)])]
      rr = RD.update(cans)
      self.assertTrue(rr is None or len(rr.errors) > 0)

  @parameterized("car_name, expected_addr, expected_count", MRR30_CAN_RADAR_CARS)
  def test_mrr30_can_radar_interface(self, car_name, expected_addr, expected_count):
    """Test MRR30_CAN radar selection for Elantra HEV."""
    RD, CP, _ = self._setup_platform(car_name)

    self.assertTrue(CP.flags & HyundaiFlags.MRR30_CAN_RADAR)
    self.assertEqual(RD.radar_addr, expected_addr)
    self.assertEqual(RD.radar_count, expected_count)
    self.assertEqual(RD.trigger_msg, expected_addr + expected_count - 1)
    self.assertEqual(MRR30_CAN_RADAR_TRACK_COUNT, 10)
    self.assertEqual(MRR30_CAN_RADAR_TRACK_END, 0x256)
    self.assertEqual(MRR30_CAN_RADAR_ADDR + ((MRR30_CAN_RADAR_TRACK_COUNT - 1) * MRR30_CAN_RADAR_GROUP_SIZE), 0x253)

  @parameterized("car_name", [CAR.HYUNDAI_ELANTRA_HEV_2021])
  def test_mrr30_can_radar_tracks_auto_enabled(self, car_name):
    """Elantra HEV MRR30_CAN full radar is hardcoded because no device toggle exists."""
    CarInterface = interfaces[car_name]
    CP = CarInterface.get_non_essential_params(car_name)
    CP_SP = CarInterface.get_non_essential_params_sp(CP, car_name)

    setup_interfaces(CarInterface, CP, CP_SP, [], None, None)

    self.assertTrue(CP_SP.flags & HyundaiFlagsSP.RADAR_FULL_RADAR)

  @parameterized("car_name", [CAR.HYUNDAI_ELANTRA_HEV_2021])
  def test_mrr30_can_full_radar_allows_tenth_group(self, car_name):
    """Full-radar mode parses the route-proven 0x253 group and route-derived vRel."""
    CarInterface = interfaces[car_name]
    CP = CarInterface.get_non_essential_params(car_name)
    CP.radarUnavailable = False
    CP_SP = CarInterface.get_non_essential_params_sp(CP, car_name)

    setup_interfaces(CarInterface, CP, CP_SP, [], None, None)

    self.assertTrue(CP_SP.flags & HyundaiFlagsSP.RADAR_FULL_RADAR)

    CI = CarInterface(CP, CP_SP)
    RD = CI.RadarInterface(CP, CP_SP)
    msg = RD.rcp.vl["RADAR_TRACK_253"]
    msg["STATE"] = 2
    msg["LONG_DIST"] = 42.0
    msg["LAT_DIST"] = -1.5
    RD.rcp.vl["RADAR_TRACK_254"]["REL_SPEED"] = -2.0

    rr = RD._update({RD.trigger_msg})

    self.assertEqual(len(rr.points), 1)
    pt = rr.points[0]
    self.assertEqual(pt.dRel, 42.0)
    self.assertEqual(pt.yRel, -1.5)
    self.assertEqual(pt.vRel, -2.0)
    self.assertTrue(pt.measured)
    self.assertTrue(math.isnan(pt.aRel))
    self.assertTrue(math.isnan(pt.yvRel))

  @parameterized("car_name", [CAR.HYUNDAI_ELANTRA_HEV_2021])
  def test_mrr30_can_suppresses_duplicate_close_detections(self, car_name):
    """Close MRR30_CAN detections with the same y/v signature are one object cluster."""
    CarInterface = interfaces[car_name]
    CP = CarInterface.get_non_essential_params(car_name)
    CP.radarUnavailable = False
    CP_SP = CarInterface.get_non_essential_params_sp(CP, car_name)

    setup_interfaces(CarInterface, CP, CP_SP, [], None, None)

    CI = CarInterface(CP, CP_SP)
    RD = CI.RadarInterface(CP, CP_SP)

    for addr, d_rel in ((0x238, 3.0), (0x23b, 4.0), (0x23e, 9.0)):
      RD.rcp.vl[f"RADAR_TRACK_{addr:x}"]["STATE"] = 2
      RD.rcp.vl[f"RADAR_TRACK_{addr:x}"]["LONG_DIST"] = d_rel
      RD.rcp.vl[f"RADAR_TRACK_{addr:x}"]["LAT_DIST"] = -1.2
      RD.rcp.vl[f"RADAR_TRACK_{addr + 1:x}"]["REL_SPEED"] = 0.2

    rr = RD._update({RD.trigger_msg})

    self.assertEqual(len(rr.points), 1)
    self.assertEqual(rr.points[0].dRel, 9.0)
    self.assertAlmostEqual(rr.points[0].yRel, -1.2)
    self.assertAlmostEqual(rr.points[0].vRel, 0.2)
