import math
from collections import deque

from opendbc.can.parser import CANParser
from opendbc.car import Bus, structs
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car.hyundai.values import CAR, DBC, HyundaiFlags

from opendbc.sunnypilot.car.hyundai.radar_interface_ext import RadarInterfaceExt
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP

MANDO_RADAR_ADDR = 0x500
RADAR_START_ADDR = MANDO_RADAR_ADDR
MANDO_RADAR_COUNT = 32
MRREVO14F_RADAR_ADDR = 0x602
MRREVO14F_RADAR_COUNT = 16
MRR30_RADAR_ADDR = 0x210
MRR30_RADAR_COUNT = 16
MRR35_RADAR_ADDR = 0x3A5
MRR35_RADAR_COUNT = 32
MRR30_CAN_RADAR_ADDR = 0x238
MRR30_CAN_RADAR_COUNT = 0x256 - MRR30_CAN_RADAR_ADDR
MRR30_CAN_RADAR_TRACK_COUNT = 10
MRR30_CAN_RADAR_GROUP_SIZE = 3
MRR30_CAN_RADAR_TRACK_END = MRR30_CAN_RADAR_ADDR + (MRR30_CAN_RADAR_TRACK_COUNT * MRR30_CAN_RADAR_GROUP_SIZE)
MRR30_CAN_RADAR_SIGNATURE = (0x238, 0x239, 0x23a, 0x255)
MRR30_CAN_RADAR_SELECTED_ADDR = 0x5ED
MRR30_CAN_RADAR_SELECTED_MSG = f"RADAR_SELECTED_{MRR30_CAN_RADAR_SELECTED_ADDR:x}"
MRR30_CAN_RADAR_SELECTED_MIN_DISTANCE = 0.0
MRR30_CAN_RADAR_SELECTED_INCREASE_HISTORY = 50
MRR30_CAN_RADAR_TAKEOFF_DISTANCE_DELTA = 1.0
MRR30_CAN_RADAR_TAKEOFF_NEGATIVE_VREL = -0.2

# POC for parsing corner radars: https://github.com/commaai/openpilot/pull/24221/


def is_mrr30_can_radar(CP):
  return CP.carFingerprint == CAR.HYUNDAI_ELANTRA_HEV_2021 and bool(CP.flags & HyundaiFlags.MRR30_CAN_RADAR)


def get_radar_can_parser(CP, radar_addr=MANDO_RADAR_ADDR, radar_count=MANDO_RADAR_COUNT, selected_addr=None, track_addrs=None):
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
    self.CP_SP = CP_SP
    self.radar_fault = False
    self.radar_done = False
    self.pts = {}

    self.mrr30_can_radar = is_mrr30_can_radar(CP)
    if self.mrr30_can_radar:
      self.radar_addr, self.radar_count = MRR30_CAN_RADAR_ADDR, MRR30_CAN_RADAR_COUNT
    elif self.CP_flags & getattr(HyundaiFlags, 'MRREVO14F_RADAR', 0):
      self.radar_addr, self.radar_count = MRREVO14F_RADAR_ADDR, MRREVO14F_RADAR_COUNT
    elif self.CP_flags & getattr(HyundaiFlags, 'MRR30_RADAR', 0):
      self.radar_addr, self.radar_count = MRR30_RADAR_ADDR, MRR30_RADAR_COUNT
    elif self.CP_flags & getattr(HyundaiFlags, 'MRR35_RADAR', 0):
      self.radar_addr, self.radar_count = MRR35_RADAR_ADDR, MRR35_RADAR_COUNT
    else:
      self.radar_addr, self.radar_count = MANDO_RADAR_ADDR, MANDO_RADAR_COUNT

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

  def _update_mrr30_can(self, ret, updated_messages):
    if not (self.CP_SP.flags & HyundaiFlagsSP.RADAR_FULL_RADAR):
      ret.points = []
      return ret

    selected_msg = self.rcp.vl[MRR30_CAN_RADAR_SELECTED_MSG]
    selected_d_rel = selected_msg["SELECTED_LONG_DIST"]
    selected_v_rel = selected_msg["SELECTED_REL_SPEED"]
    selected_valid = math.isfinite(selected_d_rel) and selected_d_rel > MRR30_CAN_RADAR_SELECTED_MIN_DISTANCE
    stale_takeoff = (
      selected_valid and self.mrr30_can_selected_d_history and
      selected_v_rel < MRR30_CAN_RADAR_TAKEOFF_NEGATIVE_VREL and
      selected_d_rel - min(self.mrr30_can_selected_d_history) > MRR30_CAN_RADAR_TAKEOFF_DISTANCE_DELTA
    )

    # 0x5ed is the radar-selected target and matches stock SCC selected distance
    # and speed. Publish every positive selected target, including the 50.2m/0mps
    # selected-radar idle point, and let radard/model matching decide whether it
    # should become a lead.
    # Stale takeoff samples are dropped before updating history; otherwise a stale
    # far point can teach the guard that the stale distance is the new baseline.
    self.pts.clear()
    if selected_valid and not stale_takeoff:
      self.mrr30_can_selected_d_history.append(selected_d_rel)
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
