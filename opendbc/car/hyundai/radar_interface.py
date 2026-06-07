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
MRR30_CAN_RADAR_SELECTED_MSG = f"RADAR_SELECTED_{MRR30_CAN_RADAR_SELECTED_ADDR:x}"
MRR30_CAN_RADAR_SELECTED_MIN_DISTANCE = 0.5
MRR30_CAN_RADAR_SELECTED_MAX_DISTANCE = 90.0
MRR30_CAN_RADAR_SELECTED_DISTANCE_BANK_SIZE = 51.2
MRR30_CAN_RADAR_SELECTED_INCREASE_HISTORY = 50
MRR30_CAN_RADAR_SELECTED_PLACEHOLDER_MIN_DISTANCE = 49.5
MRR30_CAN_RADAR_SELECTED_PLACEHOLDER_MAX_DISTANCE = 50.8
MRR30_CAN_RADAR_SELECTED_PLACEHOLDER_MAX_VREL = 0.15
MRR30_CAN_RADAR_TAKEOFF_DISTANCE_DELTA = 1.0
MRR30_CAN_RADAR_TAKEOFF_NEGATIVE_VREL = -0.2

# POC for parsing corner radars: https://github.com/commaai/openpilot/pull/24221/


def is_mrr30_can_radar(CP):
  return CP.carFingerprint == CAR.HYUNDAI_ELANTRA_HEV_2021 and bool(CP.flags & HyundaiFlags.MRR30_CAN_RADAR)


def get_radar_can_parser(CP, radar_addr=RADAR_START_ADDR, radar_count=RADAR_MSG_COUNT, selected_addr=None, track_addrs=None):
  if Bus.radar not in DBC[CP.carFingerprint]:
    return None

  track_addrs = range(radar_addr, radar_addr + radar_count) if track_addrs is None else track_addrs
  messages = [(f"RADAR_TRACK_{addr:x}", 50) for addr in track_addrs]
  if selected_addr is not None:
    messages.append((f"RADAR_SELECTED_{selected_addr:x}", 20))
  return CANParser(DBC[CP.carFingerprint][Bus.radar], messages, 1)


class RadarInterface(RadarInterfaceBase, RadarInterfaceExt):
  def __init__(self, CP, CP_SP):
    RadarInterfaceBase.__init__(self, CP, CP_SP)
    RadarInterfaceExt.__init__(self, CP, CP_SP)
    self.CP_flags = CP.flags
    self.mrr30_can_radar = is_mrr30_can_radar(CP)
    if self.mrr30_can_radar:
      self.radar_addr, self.radar_count = MRR30_CAN_RADAR_ADDR, MRR30_CAN_RADAR_COUNT
    else:
      self.radar_addr, self.radar_count = RADAR_START_ADDR, RADAR_MSG_COUNT

    self.updated_messages = set()
    self.trigger_msg = MRR30_CAN_RADAR_SELECTED_ADDR if self.mrr30_can_radar else self.radar_addr + self.radar_count - 1
    self.track_id = 0
    self.mrr30_can_selected_d_history = deque(maxlen=MRR30_CAN_RADAR_SELECTED_INCREASE_HISTORY)

    self.radar_off_can = CP.radarUnavailable
    selected_addr = MRR30_CAN_RADAR_SELECTED_ADDR if self.mrr30_can_radar else None
    track_addrs = () if self.mrr30_can_radar else None
    self.rcp = get_radar_can_parser(CP, self.radar_addr, self.radar_count, selected_addr, track_addrs)

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

    if self.mrr30_can_radar:
      return self._update_mrr30_can(ret, updated_messages)

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

  @staticmethod
  def _mrr30_can_selected_distance(selected_msg):
    # 0x5ed selected distance uses a 13-bit low value plus a route-proven bank
    # bit. Without the bank, leads beyond 51.2m wrap back to near distances.
    return selected_msg["SELECTED_LONG_DIST_LOW"] + (selected_msg["SELECTED_LONG_DIST_BANK"] * MRR30_CAN_RADAR_SELECTED_DISTANCE_BANK_SIZE)

  def _update_mrr30_can(self, ret, updated_messages):
    if not (self.CP_SP.flags & HyundaiFlagsSP.RADAR_FULL_RADAR):
      ret.points = []
      return ret

    selected_msg = self.rcp.vl[MRR30_CAN_RADAR_SELECTED_MSG]
    selected_d_rel = self._mrr30_can_selected_distance(selected_msg)
    selected_v_rel = selected_msg["SELECTED_REL_SPEED"]
    selected_valid = MRR30_CAN_RADAR_SELECTED_MIN_DISTANCE < selected_d_rel < MRR30_CAN_RADAR_SELECTED_MAX_DISTANCE
    selected_placeholder = (
      MRR30_CAN_RADAR_SELECTED_PLACEHOLDER_MIN_DISTANCE <= selected_d_rel <= MRR30_CAN_RADAR_SELECTED_PLACEHOLDER_MAX_DISTANCE and
      abs(selected_v_rel) <= MRR30_CAN_RADAR_SELECTED_PLACEHOLDER_MAX_VREL
    )
    stale_takeoff = False
    if selected_valid and selected_v_rel < MRR30_CAN_RADAR_TAKEOFF_NEGATIVE_VREL and self.mrr30_can_selected_d_history:
      stale_takeoff = selected_d_rel - min(self.mrr30_can_selected_d_history) > MRR30_CAN_RADAR_TAKEOFF_DISTANCE_DELTA

    # 0x5ed is the radar-selected target and matches stock SCC selected distance
    # and speed. 50.2m/0mps is a no-lead placeholder; it should not publish a
    # RadarPoint, but it clears history so the next real far target is fresh.
    self.pts.clear()
    if selected_placeholder:
      self.mrr30_can_selected_d_history.clear()
    elif selected_valid:
      if MRR30_CAN_RADAR_SELECTED_ADDR in updated_messages:
        self.mrr30_can_selected_d_history.append(selected_d_rel)
      if not stale_takeoff:
        point = structs.RadarData.RadarPoint()
        point.trackId = 0
        point.measured = True
        point.dRel = selected_d_rel
        point.yRel = 0.0
        point.vRel = selected_v_rel
        point.aRel = float('nan')
        point.yvRel = float('nan')
        self.pts[MRR30_CAN_RADAR_SELECTED_ADDR] = point

    ret.points = list(self.pts.values())
    return ret
