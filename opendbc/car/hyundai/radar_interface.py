import math
from collections import deque

from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car.hyundai.values import CAR, DBC, HyundaiFlags

from opendbc.sunnypilot.car.hyundai.radar_interface_ext import RadarInterfaceExt
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP

RADAR_START_ADDR = 0x500
RADAR_MSG_COUNT = 32
MRR30_CAN_RADAR_ADDR = 0x238
MRR30_CAN_RADAR_COUNT = 0x256 - MRR30_CAN_RADAR_ADDR
MRR30_CAN_RADAR_TRACK_COUNT = 10
MRR30_CAN_RADAR_GROUP_SIZE = 3
MRR30_CAN_RADAR_TRACK_END = MRR30_CAN_RADAR_ADDR + (MRR30_CAN_RADAR_TRACK_COUNT * MRR30_CAN_RADAR_GROUP_SIZE)
MRR30_CAN_RADAR_SIGNATURE = (0x238, 0x239, 0x23a, 0x255)
MRR30_CAN_RADAR_SELECTED_ADDR = 0x5ED
MRR30_CAN_RADAR_LONG_DIST_OFFSET = 3.0
MRR30_CAN_RADAR_LAT_DIST_OFFSET = 1.2
MRR30_CAN_RADAR_SELECTED_DISTANCE_TOLERANCE = 3.0
MRR30_CAN_RADAR_SELECTED_LATERAL_TOLERANCE = 3.0
MRR30_CAN_RADAR_SELECTED_INCREASE_HISTORY = 50
MRR30_CAN_RADAR_TAKEOFF_DISTANCE_DELTA = 1.0
MRR30_CAN_RADAR_TAKEOFF_NEGATIVE_VREL = -0.2

# POC for parsing corner radars: https://github.com/commaai/openpilot/pull/24221/


def get_radar_can_parser(CP, radar_addr=RADAR_START_ADDR, radar_count=RADAR_MSG_COUNT, selected_addr=None):
  if Bus.radar not in DBC[CP.carFingerprint]:
    return None

  messages = [(f"RADAR_TRACK_{addr:x}", 50) for addr in range(radar_addr, radar_addr + radar_count)]
  if selected_addr is not None:
    messages.append((f"RADAR_SELECTED_{selected_addr:x}", 20))
  return CANParser(DBC[CP.carFingerprint][Bus.radar], messages, 1)


class RadarInterface(RadarInterfaceBase, RadarInterfaceExt):
  def __init__(self, CP, CP_SP):
    RadarInterfaceBase.__init__(self, CP, CP_SP)
    RadarInterfaceExt.__init__(self, CP, CP_SP)
    self.CP_flags = CP.flags
    if self.CP_flags & HyundaiFlags.MRR30_CAN_RADAR and CP.carFingerprint == CAR.HYUNDAI_ELANTRA_HEV_2021:
      self.radar_addr, self.radar_count = MRR30_CAN_RADAR_ADDR, MRR30_CAN_RADAR_COUNT
    else:
      self.radar_addr, self.radar_count = RADAR_START_ADDR, RADAR_MSG_COUNT

    self.updated_messages = set()
    self.trigger_msg = self.radar_addr + self.radar_count - 1
    self.track_id = 0
    self.mrr30_can_selected_d_history = deque(maxlen=MRR30_CAN_RADAR_SELECTED_INCREASE_HISTORY)

    self.radar_off_can = CP.radarUnavailable
    selected_addr = MRR30_CAN_RADAR_SELECTED_ADDR if self.CP_flags & HyundaiFlags.MRR30_CAN_RADAR and CP.carFingerprint == CAR.HYUNDAI_ELANTRA_HEV_2021 else None
    self.rcp = get_radar_can_parser(CP, self.radar_addr, self.radar_count, selected_addr)

    if self.rcp is None:
      self.initialize_radar_ext(self.trigger_msg)

  def update(self, can_strings):
    if self.radar_off_can or (self.rcp is None):
      return super().update(None)

    vls = self.rcp.update(can_strings)
    self.updated_messages.update(vls)

    if self.trigger_msg not in self.updated_messages:
      return None

    rr = self._update(self.updated_messages)
    self.updated_messages.clear()

    return rr

  def _update(self, updated_messages):
    ret = structs.RadarData()
    if self.rcp is None:
      return ret

    if not self.rcp.can_valid:
      ret.errors.canError = True

    if self.use_radar_interface_ext:
      return self.update_ext(ret)

    if self.CP_flags & HyundaiFlags.MRR30_CAN_RADAR and self.CP.carFingerprint == CAR.HYUNDAI_ELANTRA_HEV_2021:
      return self._update_mrr30_can(ret)

    for addr in range(self.radar_addr, self.radar_addr + self.radar_count):
      msg = self.rcp.vl[f"RADAR_TRACK_{addr:x}"]
      if addr not in self.pts:
        self.pts[addr] = structs.RadarData.RadarPoint()
        self.pts[addr].trackId = self.track_id
        self.track_id += 1

      valid = msg['STATE'] in (3, 4)
      if valid:
        azimuth = math.radians(msg['AZIMUTH'])
        self.pts[addr].measured = True
        self.pts[addr].dRel = math.cos(azimuth) * msg['LONG_DIST']
        self.pts[addr].yRel = 0.5 * -math.sin(azimuth) * msg['LONG_DIST']
        self.pts[addr].vRel = msg['REL_SPEED']
        self.pts[addr].aRel = msg['REL_ACCEL']
        self.pts[addr].yvRel = float('nan')

      else:
        del self.pts[addr]

    ret.points = list(self.pts.values())
    return ret

  def _update_mrr30_can(self, ret):
    if not self.CP_SP.flags & HyundaiFlagsSP.RADAR_FULL_RADAR:
      ret.points = []
      return ret

    selected_msg = self.rcp.vl[f"RADAR_SELECTED_{MRR30_CAN_RADAR_SELECTED_ADDR:x}"]
    selected_d_rel = selected_msg["SELECTED_LONG_DIST"]
    selected_v_rel = selected_msg["SELECTED_REL_SPEED"]
    selected_valid = selected_d_rel > 0.5
    selected_inconsistent_takeoff = (
      selected_valid and selected_v_rel < MRR30_CAN_RADAR_TAKEOFF_NEGATIVE_VREL and
      len(self.mrr30_can_selected_d_history) and
      selected_d_rel - min(self.mrr30_can_selected_d_history) > MRR30_CAN_RADAR_TAKEOFF_DISTANCE_DELTA
    )
    if selected_valid:
      self.mrr30_can_selected_d_history.append(selected_d_rel)

    best_msg = None
    best_diff = math.inf
    for addr in range(MRR30_CAN_RADAR_ADDR, MRR30_CAN_RADAR_TRACK_END, MRR30_CAN_RADAR_GROUP_SIZE):
      msg = self.rcp.vl[f"RADAR_TRACK_{addr:x}"]
      d_rel = msg['LONG_DIST'] + MRR30_CAN_RADAR_LONG_DIST_OFFSET
      y_rel = msg['LAT_DIST'] + MRR30_CAN_RADAR_LAT_DIST_OFFSET
      selected_diff = abs(d_rel - selected_d_rel)
      selected_match = (
        selected_valid and not selected_inconsistent_takeoff and
        selected_diff <= MRR30_CAN_RADAR_SELECTED_DISTANCE_TOLERANCE and
        abs(y_rel) <= MRR30_CAN_RADAR_SELECTED_LATERAL_TOLERANCE
      )
      if msg['STATE'] == 2 and msg['LONG_DIST'] > 0 and selected_match:
        if selected_diff < best_diff:
          best_msg = msg
          best_diff = selected_diff

    self.pts.clear()
    if best_msg is not None:
      self.pts[MRR30_CAN_RADAR_SELECTED_ADDR] = structs.RadarData.RadarPoint()
      self.pts[MRR30_CAN_RADAR_SELECTED_ADDR].trackId = 0
      self.pts[MRR30_CAN_RADAR_SELECTED_ADDR].measured = True
      self.pts[MRR30_CAN_RADAR_SELECTED_ADDR].dRel = selected_d_rel
      self.pts[MRR30_CAN_RADAR_SELECTED_ADDR].yRel = best_msg['LAT_DIST'] + MRR30_CAN_RADAR_LAT_DIST_OFFSET
      self.pts[MRR30_CAN_RADAR_SELECTED_ADDR].vRel = selected_v_rel
      self.pts[MRR30_CAN_RADAR_SELECTED_ADDR].aRel = float('nan')
      self.pts[MRR30_CAN_RADAR_SELECTED_ADDR].yvRel = float('nan')

    ret.points = list(self.pts.values())
    return ret
