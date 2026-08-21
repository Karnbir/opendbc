import math

from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car.hyundai.values import DBC, HyundaiFlags

from opendbc.sunnypilot.car.hyundai.radar_interface_ext import RadarInterfaceExt

RADAR_START_ADDR = 0x500
RADAR_MSG_COUNT = 32
SCC_SELECTED_LEAD_ADDR = 0x5ED
SCC_SELECTED_LEAD_MSG = f"SCC_SELECTED_LEAD_{SCC_SELECTED_LEAD_ADDR:x}"
SCC_SELECTED_LEAD_IDLE_DISTANCE = 50.2
# The no-lead value varies by a few 6.25 mm distance bins in recorded routes.
SCC_SELECTED_LEAD_IDLE_DISTANCE_TOLERANCE = 0.02
SCC_SELECTED_LEAD_IDLE_SPEED_TOLERANCE = 0.05

# POC for parsing corner radars: https://github.com/commaai/openpilot/pull/24221/


def get_radar_can_parser(CP):
  if Bus.radar not in DBC[CP.carFingerprint]:
    return None

  if CP.flags & HyundaiFlags.SCC_SELECTED_LEAD:
    messages = [(SCC_SELECTED_LEAD_MSG, 20)]
  else:
    messages = [(f"RADAR_TRACK_{addr:x}", 50) for addr in range(RADAR_START_ADDR, RADAR_START_ADDR + RADAR_MSG_COUNT)]
  return CANParser(DBC[CP.carFingerprint][Bus.radar], messages, 1)


class RadarInterface(RadarInterfaceBase, RadarInterfaceExt):
  def __init__(self, CP, CP_SP):
    RadarInterfaceBase.__init__(self, CP, CP_SP)
    RadarInterfaceExt.__init__(self, CP, CP_SP)
    self.scc_selected_lead = bool(CP.flags & HyundaiFlags.SCC_SELECTED_LEAD)
    self.updated_messages = set()
    self.trigger_msg = SCC_SELECTED_LEAD_ADDR if self.scc_selected_lead else RADAR_START_ADDR + RADAR_MSG_COUNT - 1

    self.radar_off_can = CP.radarUnavailable
    self.rcp = get_radar_can_parser(CP)

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

    if self.scc_selected_lead:
      return self._update_scc_selected_lead(ret)

    for addr in range(RADAR_START_ADDR, RADAR_START_ADDR + RADAR_MSG_COUNT):
      msg = self.rcp.vl[f"RADAR_TRACK_{addr:x}"]

      if addr not in self.pts:
        self.pts[addr] = structs.RadarData.RadarPoint()
        self.pts[addr].trackId = self.track_id
        self.track_id += 1

      valid = msg['STATE'] in (3, 4)
      if valid:
        azimuth = math.radians(msg['AZIMUTH'])
        self.pts[addr].dRel = math.cos(azimuth) * msg['LONG_DIST']
        self.pts[addr].yRel = 0.5 * -math.sin(azimuth) * msg['LONG_DIST']
        self.pts[addr].vRel = msg['REL_SPEED']

      else:
        del self.pts[addr]

    ret.points = list(self.pts.values())
    return ret

  def _update_scc_selected_lead(self, ret):
    msg = self.rcp.vl[SCC_SELECTED_LEAD_MSG]
    d_rel = msg["SELECTED_LONG_DIST"]
    v_rel = msg["SELECTED_REL_SPEED"]
    idle = (
      abs(d_rel - SCC_SELECTED_LEAD_IDLE_DISTANCE) <= SCC_SELECTED_LEAD_IDLE_DISTANCE_TOLERANCE and
      abs(v_rel) <= SCC_SELECTED_LEAD_IDLE_SPEED_TOLERANCE
    )

    # 0x5ED is the stock SCC-selected target; 50.2 m at 0 m/s is its no-lead value.
    if not idle:
      point = structs.RadarData.RadarPoint()
      point.trackId = 0
      point.dRel = d_rel
      point.yRel = 0.0
      point.vRel = v_rel
      ret.points = [point]

    return ret
