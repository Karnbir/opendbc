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
MRR30_CAN_RADAR_TRACK_ADDRS = tuple(range(MRR30_CAN_RADAR_ADDR, MRR30_CAN_RADAR_TRACK_END, MRR30_CAN_RADAR_GROUP_SIZE))
MRR30_CAN_RADAR_SIGNATURE = (0x238, 0x239, 0x23a, 0x255)
MRR30_CAN_RADAR_SELECTED_ADDR = 0x5ED
MRR30_CAN_RADAR_SELECTED_MSG = f"RADAR_SELECTED_{MRR30_CAN_RADAR_SELECTED_ADDR:x}"
MRR30_CAN_RADAR_VALID_STATE = 2
MRR30_CAN_RADAR_LONG_DIST_OFFSET = 3.0
MRR30_CAN_RADAR_LAT_DIST_OFFSET = 1.2
MRR30_CAN_RADAR_SELECTED_MIN_DISTANCE = 0.5
MRR30_CAN_RADAR_SELECTED_MAX_DISTANCE = 49.0
MRR30_CAN_RADAR_SELECTED_DISTANCE_TOLERANCE = 3.0
MRR30_CAN_RADAR_SELECTED_LATERAL_TOLERANCE = 3.0
MRR30_CAN_RADAR_SELECTED_INCREASE_HISTORY = 50
MRR30_CAN_RADAR_SELECTED_UPDATE_DT = 0.05
MRR30_CAN_RADAR_SELECTED_VREL_ALPHA = 0.5
MRR30_CAN_RADAR_SELECTED_VREL_MAX = 4.0
MRR30_CAN_RADAR_TAKEOFF_DISTANCE_DELTA = 1.0
MRR30_CAN_RADAR_TAKEOFF_NEGATIVE_VREL = -0.2

# POC for parsing corner radars: https://github.com/commaai/openpilot/pull/24221/


def is_mrr30_can_radar(CP):
  return CP.carFingerprint == CAR.HYUNDAI_ELANTRA_HEV_2021 and bool(CP.flags & HyundaiFlags.MRR30_CAN_RADAR)


def mrr30_can_radar_track_msg_name(addr):
  return f"RADAR_TRACK_{addr:x}"


def mrr30_can_radar_point_from_track(msg):
  return msg['LONG_DIST'] + MRR30_CAN_RADAR_LONG_DIST_OFFSET, msg['LAT_DIST'] + MRR30_CAN_RADAR_LAT_DIST_OFFSET


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
    self.mrr30_can_radar = is_mrr30_can_radar(CP)
    if self.mrr30_can_radar:
      self.radar_addr, self.radar_count = MRR30_CAN_RADAR_ADDR, MRR30_CAN_RADAR_COUNT
    else:
      self.radar_addr, self.radar_count = RADAR_START_ADDR, RADAR_MSG_COUNT

    self.updated_messages = set()
    self.trigger_msg = self.radar_addr + self.radar_count - 1
    self.track_id = 0
    self.mrr30_can_selected_d_history = deque(maxlen=MRR30_CAN_RADAR_SELECTED_INCREASE_HISTORY)
    self.mrr30_can_selected_prev_d = math.nan
    self.mrr30_can_selected_v_rel = 0.0

    self.radar_off_can = CP.radarUnavailable
    selected_addr = MRR30_CAN_RADAR_SELECTED_ADDR if self.mrr30_can_radar else None
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

  def _update_mrr30_can_selected_v_rel(self, selected_d_rel, selected_valid, selected_updated):
    if not selected_valid:
      self.mrr30_can_selected_prev_d = math.nan
      self.mrr30_can_selected_v_rel = 0.0
      return self.mrr30_can_selected_v_rel

    if not selected_updated:
      return self.mrr30_can_selected_v_rel

    if math.isfinite(self.mrr30_can_selected_prev_d):
      # The selected speed byte is useful as a stale-lead sanity check, but route
      # data showed better longitudinal stability from the selected distance rate.
      v_rel = (selected_d_rel - self.mrr30_can_selected_prev_d) / MRR30_CAN_RADAR_SELECTED_UPDATE_DT
      v_rel = max(-MRR30_CAN_RADAR_SELECTED_VREL_MAX, min(MRR30_CAN_RADAR_SELECTED_VREL_MAX, v_rel))
      self.mrr30_can_selected_v_rel += MRR30_CAN_RADAR_SELECTED_VREL_ALPHA * (v_rel - self.mrr30_can_selected_v_rel)

    self.mrr30_can_selected_prev_d = selected_d_rel
    return self.mrr30_can_selected_v_rel

  def _mrr30_can_selected_valid(self, selected_d_rel):
    return MRR30_CAN_RADAR_SELECTED_MIN_DISTANCE < selected_d_rel < MRR30_CAN_RADAR_SELECTED_MAX_DISTANCE

  def _mrr30_can_selected_inconsistent_takeoff(self, selected_d_rel, selected_v_rel_raw, selected_valid):
    if not selected_valid or selected_v_rel_raw >= MRR30_CAN_RADAR_TAKEOFF_NEGATIVE_VREL or not self.mrr30_can_selected_d_history:
      return False
    return selected_d_rel - min(self.mrr30_can_selected_d_history) > MRR30_CAN_RADAR_TAKEOFF_DISTANCE_DELTA

  def _mrr30_can_raw_track_valid(self, msg):
    return msg['STATE'] == MRR30_CAN_RADAR_VALID_STATE and msg['LONG_DIST'] > 0

  def _mrr30_can_matching_selected_track(self, selected_d_rel, selected_valid, selected_inconsistent_takeoff):
    if not selected_valid or selected_inconsistent_takeoff:
      return None

    best_msg = None
    best_diff = math.inf
    for addr in MRR30_CAN_RADAR_TRACK_ADDRS:
      msg = self.rcp.vl[mrr30_can_radar_track_msg_name(addr)]
      if not self._mrr30_can_raw_track_valid(msg):
        continue

      d_rel, y_rel = mrr30_can_radar_point_from_track(msg)
      selected_diff = abs(d_rel - selected_d_rel)
      if selected_diff > MRR30_CAN_RADAR_SELECTED_DISTANCE_TOLERANCE or abs(y_rel) > MRR30_CAN_RADAR_SELECTED_LATERAL_TOLERANCE:
        continue

      if selected_diff < best_diff:
        best_msg = msg
        best_diff = selected_diff

    return best_msg

  def _mrr30_can_publish_selected_point(self, selected_d_rel, track_msg, selected_v_rel):
    point = structs.RadarData.RadarPoint()
    point.trackId = 0
    point.measured = True
    point.dRel = selected_d_rel
    _, point.yRel = mrr30_can_radar_point_from_track(track_msg)
    point.vRel = selected_v_rel
    point.aRel = float('nan')
    point.yvRel = float('nan')
    self.pts[MRR30_CAN_RADAR_SELECTED_ADDR] = point

  def _update_mrr30_can(self, ret, updated_messages):
    if not (self.CP_SP.flags & HyundaiFlagsSP.RADAR_FULL_RADAR):
      ret.points = []
      return ret

    selected_msg = self.rcp.vl[MRR30_CAN_RADAR_SELECTED_MSG]
    selected_d_rel = selected_msg["SELECTED_LONG_DIST"]
    selected_v_rel_raw = selected_msg["SELECTED_REL_SPEED"]
    selected_valid = self._mrr30_can_selected_valid(selected_d_rel)
    selected_updated = MRR30_CAN_RADAR_SELECTED_ADDR in updated_messages
    selected_inconsistent_takeoff = self._mrr30_can_selected_inconsistent_takeoff(selected_d_rel, selected_v_rel_raw, selected_valid)
    selected_v_rel = self._update_mrr30_can_selected_v_rel(selected_d_rel, selected_valid, selected_updated)
    if selected_valid and selected_updated:
      self.mrr30_can_selected_d_history.append(selected_d_rel)

    # 0x23x raw tracks can include side/static returns. 0x5ed is the radar's
    # selected target, so publish one point only when a route-proven raw track
    # agrees with that selected target.
    best_msg = self._mrr30_can_matching_selected_track(selected_d_rel, selected_valid, selected_inconsistent_takeoff)
    self.pts.clear()
    if best_msg is not None:
      self._mrr30_can_publish_selected_point(selected_d_rel, best_msg, selected_v_rel)

    ret.points = list(self.pts.values())
    return ret
