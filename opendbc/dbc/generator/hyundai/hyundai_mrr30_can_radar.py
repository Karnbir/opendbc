#!/usr/bin/env python3


REL_SPEED_FACTOR = 0.016
SELECTED_DISTANCE_FACTOR = 0.00625
SELECTED_REL_SPEED_FACTOR = 0.1


def generate():
  parts = []
  parts.append("""
VERSION ""


NS_ :
    NS_DESC_
    CM_
    BA_DEF_
    BA_
    VAL_
    CAT_DEF_
    CAT_
    FILTER
    BA_DEF_DEF_
    EV_DATA_
    ENVVAR_DATA_
    SGTYPE_
    SGTYPE_VAL_
    BA_DEF_SGTYPE_
    BA_SGTYPE_
    SIG_TYPE_REF_
    VAL_TABLE_
    SIG_GROUP_
    SIG_VALTYPE_
    SIGTYPE_VALTYPE_
    BO_TX_BU_
    BA_DEF_REL_
    BA_REL_
    BA_DEF_DEF_REL_
    BU_SG_REL_
    BU_EV_REL_
    BU_BO_REL_
    SG_MUL_VAL_

BS_:

BU_: XXX
    """)

  # MRR30_CAN emits one raw object group across three 8-byte messages. The
  # route-proven groups are 0x238-0x255. These raw fields are kept for
  # Cabana/debug only; Elantra HEV publishes the SCC-selected 0x5ed lead as its
  # RadarPoint. Raw-track REL_SPEED, acceleration, and lateral velocity remain
  # analysis-only.
  #
  # SELECTED_LONG_DIST combines the 13-bit lower distance and the 1-bit bank
  # (which is bit 17) into a single 14-bit signal spanning up to 102.39m.
  # SELECTED_REL_SPEED starts after that bank bit at bit 19.
  for addr in range(0x238, 0x256, 3):
    parts.append(f"""
BO_ {addr} RADAR_TRACK_{addr:x}: 8 RADAR
 SG_ LONG_DIST : 6|7@0+ (1,0) [0|127] "m" XXX
 SG_ STATE : 9|2@0+ (1,0) [0|3] "" XXX
 SG_ LAT_DIST : 55|8@0- (0.1,0) [-12.8|12.7] "m" XXX

BO_ {addr + 1} RADAR_TRACK_{addr + 1:x}: 8 RADAR
 SG_ UNKNOWN_1 : 6|10@1+ (1,0) [0|1023] "" XXX
 SG_ UNKNOWN_2 : 16|10@1+ (1,0) [0|1023] "" XXX
 SG_ UNKNOWN_4 : 42|2@1+ (1,0) [0|3] "" XXX
 SG_ REL_SPEED : 44|12@1- ({REL_SPEED_FACTOR},0) [-32.768|32.752] "m/s" XXX

BO_ {addr + 2} RADAR_TRACK_{addr + 2:x}: 8 RADAR
 SG_ UNKNOWN_6 : 10|6@1+ (1,0) [0|63] "" XXX
 SG_ COUNTER : 31|8@0+ (1,0) [0|255] "" XXX
 SG_ UNKNOWN_1 : 32|12@1+ (1,0) [0|4095] "" XXX
 SG_ UNKNOWN_3 : 45|7@1+ (1,0) [0|127] "" XXX
 SG_ UNKNOWN_4 : 54|4@1+ (1,0) [0|15] "" XXX
 SG_ UNKNOWN_5 : 63|6@0+ (1,0) [0|63] "" XXX
    """)

  parts.append(f"""
 BO_ 1517 RADAR_SELECTED_5ed: 8 RADAR
  SG_ ALIVE_COUNTER : 0|4@1+ (1,0) [0|15] "" XXX
  SG_ SELECTED_LONG_DIST : 4|14@1+ ({SELECTED_DISTANCE_FACTOR},0) [0|102.39375] "m" XXX
  SG_ NEW_TRACK : 18|1@1+ (1,0) [0|1] "" XXX
  SG_ SELECTED_REL_SPEED : 19|11@1- ({SELECTED_REL_SPEED_FACTOR},0) [-102.4|102.3] "m/s" XXX
    """)

  return {"hyundai_mrr30_can_radar.dbc": "".join(parts)}
