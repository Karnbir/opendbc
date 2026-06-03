import math

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
MRR30_CAN_RADAR_DUPLICATE_DIST = 15.0
MRR30_CAN_RADAR_DUPLICATE_Y_TOL = 0.35
MRR30_CAN_RADAR_DUPLICATE_V_TOL = 0.35

# POC for parsing corner radars: https://github.com/commaai/openpilot/pull/24221/


def get_radar_can_parser(CP, radar_addr=RADAR_START_ADDR, radar_count=RADAR_MSG_COUNT):
  if Bus.radar not in DBC[CP.carFingerprint]:
    return None

  messages = [(f"RADAR_TRACK_{addr:x}", 50) for addr in range(radar_addr, radar_addr + radar_count)]
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

    self.radar_off_can = CP.radarUnavailable
    self.rcp = get_radar_can_parser(CP, self.radar_addr, self.radar_count)

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

    candidates = {}
    for addr in range(MRR30_CAN_RADAR_ADDR, MRR30_CAN_RADAR_TRACK_END, MRR30_CAN_RADAR_GROUP_SIZE):
      msg = self.rcp.vl[f"RADAR_TRACK_{addr:x}"]
      rel_speed_msg = self.rcp.vl[f"RADAR_TRACK_{addr + 1:x}"]
      if msg['STATE'] == 2 and msg['LONG_DIST'] > 0:
        candidates[addr] = (msg['LONG_DIST'], msg['LAT_DIST'], rel_speed_msg['REL_SPEED'])

    suppressed_addrs = self._get_mrr30_can_duplicate_addrs(candidates)

    for addr in range(MRR30_CAN_RADAR_ADDR, MRR30_CAN_RADAR_TRACK_END, MRR30_CAN_RADAR_GROUP_SIZE):
      if addr not in candidates or addr in suppressed_addrs:
        self.pts.pop(addr, None)
        continue

      d_rel, y_rel, v_rel = candidates[addr]
      if addr not in self.pts:
        self.pts[addr] = structs.RadarData.RadarPoint()
        self.pts[addr].trackId = self.track_id
        self.track_id += 1

      self.pts[addr].measured = True
      self.pts[addr].dRel = d_rel
      self.pts[addr].yRel = y_rel
      self.pts[addr].vRel = v_rel
      self.pts[addr].aRel = float('nan')
      self.pts[addr].yvRel = float('nan')

    ret.points = list(self.pts.values())
    return ret

  @staticmethod
  def _get_mrr30_can_duplicate_addrs(candidates):
    suppressed_addrs = set()
    addrs = list(candidates)

    for addr in addrs:
      if addr in suppressed_addrs:
        continue

      d_rel, y_rel, v_rel = candidates[addr]
      if d_rel > MRR30_CAN_RADAR_DUPLICATE_DIST:
        continue

      duplicate_addrs = [addr]
      for other_addr in addrs:
        if other_addr == addr or other_addr in suppressed_addrs:
          continue

        other_d_rel, other_y_rel, other_v_rel = candidates[other_addr]
        if other_d_rel > MRR30_CAN_RADAR_DUPLICATE_DIST:
          continue
        if abs(y_rel - other_y_rel) <= MRR30_CAN_RADAR_DUPLICATE_Y_TOL and \
           abs(v_rel - other_v_rel) <= MRR30_CAN_RADAR_DUPLICATE_V_TOL:
          duplicate_addrs.append(other_addr)

      if len(duplicate_addrs) > 1:
        keep_addr = max(duplicate_addrs, key=lambda candidate_addr: candidates[candidate_addr][0])
        suppressed_addrs.update(a for a in duplicate_addrs if a != keep_addr)

    return suppressed_addrs
