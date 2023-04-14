#!/usr/bin/env python
# (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

import argparse
import os
import re
import socket
import struct
import sys
import threading
import time

import usb.core
import usb.util
import yaml

# Minimum FW versions this script supports
MIN_FW_VER_MAJOR = 0
MIN_FW_VER_MINOR = 6
MIN_FW_DAISY_CHAIN_VER_MINOR = 7

# Version that this script is
# TODO: Replace 'FW' with 'SCRIPT' in these constants and in release scripts.
FW_VER_MAJOR = 0
FW_VER_MINOR = 0

# Text UI display settings
MAX_NAME_PRINT_LEN = 22
NUM_COLUMNS = 4
NUM_DECIMAL_PLACES = 4

# Sentinel USB device constants
USB_TO_DEV_CFG_EP = 1
USB_FROM_DEV_CFG_EP = 0x81
USB_FROM_DEV_DATA_EP = 0x82

SENTINEL_USB_VID = 0x2833
SENTINEL_USB_PID = 0x0203

###########################################################
# MUST MATCH! See src/usb_sentinel.h for enum definitions
###########################################################
SENTINEL_BOARD_NONE = 0
SENTINEL_BOARD_DELPHI_NFF0 = 1
SENTINEL_BOARD_STANDALONE = 2
board_names = ["None", "Delphi NFF0", "Standalone"]
INA228 = 228
INA_NONE = -1

INA_MODE_VSHUNT = 0
INA_MODE_VBUS = 1
INA_MODE_DIETEMP = 2
INA_MODE_CURRENT = 3
INA_MODE_POWER = 4
INA_MODE_ENERGY = 5
INA_MODE_CHARGE = 6
INA_MODE_GPIO = 7

ALL_BUSSES = 0
MAX_BOARDS = 4  # Max supported daisy chain length
MAX_S_RAIL = 3  # Max "subrails" per rail for alternate register readouts

GPIO_BUS_ID = 0xFF

# USB message IDs
SENTINEL_MSG_NONE = 0
SENTINEL_MSG_BOARD_INFO_REQ = 1
SENTINEL_MSG_BOARD_INFO_RSP = 2
SENTINEL_MSG_RAIL_CFG = 3
SENTINEL_MSG_RAIL_CFG_RSP = 4
SENTINEL_MSG_DATA_STREAM = 5
SENTINEL_MSG_I2C_WRITE_READ = 6
SENTINEL_MSG_I2C_WRITE_READ_RSP = 7
SENTINEL_MSG_ENTER_BOOTLOADER = 8
SENTINEL_MSG_ENTER_BOOTLOADER_RSP = 9
SENTINEL_MSG_REBOOT = 10
SENTINEL_MSG_CFG_RESET = 11
SENTINEL_MSG_STREAMING_ON_OFF = 12
SENTINEL_MSG_INA_RESET = 13
SENTINEL_MSG_INA_TEST = 14
SENTINEL_MSG_SET_TIMESTAMP = 15
SENTINEL_MSG_USB_TUN_START = 254
SENTINEL_MSG_MAX = 255

# From usb_sentinel.h,
# typedef enum { I2C_ERR, BUFF_OVRRNS, SCHD_MISS, NUM_ERROR_TYPE } ERROR_TYPE
SENTINEL_MSG_ERROR_TYPES = ["I2C_ERR", "BUFF_OVRRNS", "SCHD_MISS"]
ERROR_XMIT_THRESHOLDS = {"I2C_ERR": 1, "BUFF_OVRRNS": 16, "SCHD_MISS": 16}

# Error count at which an error is emitted on STDOUT
ERROR_COUNT_THRESHOLD = 1024

# Top-line warning messages displayed above text UI
static_msgs = []
board_errors = {}

ina_mode = {
    "VSHUNT": INA_MODE_VSHUNT,
    "VBUS": INA_MODE_VBUS,
    "DIETEMP": INA_MODE_DIETEMP,
    "CURRENT": INA_MODE_CURRENT,
    "POWER": INA_MODE_POWER,
    "ENERGY": INA_MODE_ENERGY,
    "CHARGE": INA_MODE_CHARGE,
    "GPIO": INA_MODE_GPIO,
}

streaming_on = True
# Number of daisy chain hops to target board when using single-USB data daisy chaining
num_hops_to_dest_board = 0
# Flag to shut down board reader worker threads
is_exiting = False
# Flag to signal that recording has begun
is_recording = False


class SentinelConfig:
    def __init__(
        self,
        kwargs={},
    ):

        self.yaml_file = kwargs.get("config", None)
        self.board_index = kwargs.get("boardindex", 0)
        self.csv_dump_file = kwargs.get("dump_sync", None)
        self.scope = kwargs.get("scope", False)
        self.ymax = kwargs.get("ymax", None)
        self.plot_samples = kwargs.get("plot_samples", 200)
        self.verbose = kwargs.get("verbose", False)
        self.sample_rate = kwargs.get("sample_rate", None)

        self.connect_pts = kwargs.get("connect_pts", None)
        self.test = kwargs.get("test", False)
        self.sampling_mode = kwargs.get("modesample", False)

        self.sync_uptimes = kwargs.get("uptimesync", False)
        self.quiet_mode = kwargs.get("quiet_mode", False)
        self.configure_only = kwargs.get("cfg_only", False)
        self.capture_only = kwargs.get("cap_only", False)
        self.abort_on_error = kwargs.get("abort_on_error", False)
        self.length_seconds = kwargs.get("length_seconds", 0)
        self.calc_stats = kwargs.get("calc_stats", False)
        self.printer = kwargs.get("printer", "standard")
        self.filter_rails = kwargs.get("filter", False)
        self.timebase = kwargs.get("timebase", "sentinel")
        self.flag_file_path = kwargs.get("flag_file_path", None)

    def set_yaml_file(self, yaml_file: str):
        self.yaml_file = yaml_file

    def set_board_index(self, board_index: int):
        if board_index >= 0:
            self.board_index = board_index

    def set_csv_dump_file(self, csv_dump_file: str):
        self.csv_dump_file = csv_dump_file

    def set_scope(self, scope):
        self.scope = scope

    def set_ymax(self, ymax: int):
        self.ymax = ymax

    def set_plot_samples(self, plot_samples: int):
        self.plot_samples = plot_samples

    def enable_verbose(self):
        self.verbose = True

    def set_sample_rate(self, sample_rate: int):
        self.sample_rate = sample_rate

    def set_connect_pts(self, connect_pts: str):
        self.connect_pts = connect_pts

    def set_test(self, test: str):
        self.test = test

    def set_sampling_mode(self, sampling_mode: str):
        self.sampling_mode = sampling_mode

    def enable_sync_uptimes(self):
        self.sync_uptimes = True

    def enable_quiet_mode(self):
        self.quiet_mode = True

    def enable_configure_only(self):
        self.configure_only = True
        self.capture_only = False

    def enable_cap_only(self):
        self.configure_only = False
        self.capture_only = True

    def eanble_abort_on_error(self):
        self.abort_on_error = True

    def set_length(self, length: int):
        if length > 0:
            self.length_seconds = length

    def enable_calc_stats(self):
        self.calc_stats = True

    def set_printer(self, printer: str):
        if (printer == "standard") or (printer == "json") or (printer == "hierarchy"):

            self.printer = printer

    def set_filter_rails(self, filt):
        self.filter_rails = filt

    def set_timebase(self, timebase):
        if timebase in ["host", "sentinel", "trigger"]:
            self.timebase = timebase


class style:
    PURPLE = "\033[95m"
    CYAN = "\033[96m"
    DARKCYAN = "\033[36m"
    BLUE = "\033[94m"
    GREEN = "\033[92m"
    YELLOW = "\033[93m"
    RED = "\033[91m"
    BOLD = "\033[1m"
    UNDERLINE = "\033[4m"
    END = "\033[0m"


def eprint(*args, **kwargs):
    # Print to stderr for error messages and test status
    print(*args, file=sys.stderr, **kwargs)


class Rail:
    def __init__(
        self,
        name,
        ina,
        mode,
        str_bus,
        str_adr,
        str_hz,
        str_ohm,
        str_maxA,
        str_cTA,
        subsystem=None,
        parent=None,
    ):

        # Sensible defaults for omitted fields
        if ina is None:
            ina = "INA228"
        if mode is None:
            # VBUS is the least demanding and most likely to work despite problems
            mode = "VBUS"
        if str_hz is None:
            str_hz = "200"

        if ina != "INA228" and ina != "GPIO":
            raise TypeError("[Rail='{}'] 'ina' invalid".format(name))
        if str_bus is None and ina != "GPIO":
            raise TypeError("[Rail='{}'] 'bus' missing".format(name))
        if str_adr is None:
            raise TypeError("[Rail='{}'] 'adr' missing".format(name))

        self.ina = ina  # INA part number (always 228, presently)
        self.str_bus = str_bus  # I2C bus number (1,2,3)
        self.str_adr = str_adr  # I2C bus address of INA
        self.str_hz = str_hz  # Sampling rate
        self.str_ohm = str_ohm  # Shunt resistance (if applicable)
        self.str_maxA = str_maxA  # Full scale amperage
        # Note: str_ohm must, combined with str_maxA, yield voltages within device limits
        self.str_cTA = str_cTA  # Manual ADC conversion time & averaging config (opt.)
        self.subsystem = subsystem  # DUT subsystem annotation (optional)
        self.parent = parent  # Parent rail (same INA) if a "subrail" assigned to an alternate INA register
        self.usbdevnum = (
            None  # Won't be known until Sentinel configures each device's rails
        )

        if ina == "GPIO" and str_bus is None:
            bus = GPIO_BUS_ID
        else:
            try:
                bus = int(str(str_bus), 0)
                assert bus == 1 or bus == 2 or bus == 3
            except Exception:
                raise ValueError(
                    "[Rail='{}'] 'bus' = '{}' invalid. Must be 1, 2, or 3".format(
                        name, str_bus
                    )
                )

        try:
            adr = int(str(str_adr), 0)
            assert adr >= 0x40 and adr < 0x50 or ina == "GPIO"
        except Exception:
            raise ValueError(
                "[Rail='{}'] 'adr' = '{}' invalid. Must be between 0x40-0x4F".format(
                    name, str_adr
                )
            )

        if str_ohm is not None:
            try:
                ohm = float(str(str_ohm))
                assert ohm > 0.0
            except Exception:
                raise ValueError(
                    "[Rail='{}'] 'ohm' = '{}' invalid. Must be > 0.0".format(
                        name, str_ohm
                    )
                )
        else:
            ohm = 0.0

        try:
            hz = int(str(str_hz), 0)
            assert hz >= 0
            self.sampleRate = str(hz)
        except Exception:
            raise ValueError(
                "[Rail='{}'] 'hz' = '{}' invalid. Must be >= 0".format(name, str_hz)
            )

        if str_maxA is not None:
            try:
                maxA = float(str(str_maxA))
                assert maxA > 0
            except Exception:
                raise ValueError(
                    "[Rail='{}'] 'maxA' = '{}' invalid. Must be > 0".format(
                        name, str_maxA
                    )
                )
        else:
            maxA = None

        cTimeAvg = None
        enableModes = None
        if str_cTA is not None:
            # Prepare ADC configuration bits for conversion time and averaging
            CT_TBL = {
                0: 0,
                50: 0,
                84: 1,
                150: 2,
                280: 3,
                540: 4,
                1052: 5,
                2074: 6,
                4120: 7,
            }
            AVG_TBL = {
                1: 0,
                4: 1,
                16: 2,
                64: 3,
                128: 4,
                256: 5,
                512: 6,
                1024: 7,
            }
            # vbct - vbus conversion time
            # vshct - vshunt conversion time
            # vtct - dietemp conversion time

            vbct = vshct = vtct = avg = 0
            try:
                vbct_str, vshct_str, vtct_str, avg_str = str_cTA.strip().split(",")
                vbct = int(vbct_str)
                vshct = int(vshct_str)
                vtct = int(vtct_str)
                avg = int(avg_str)
            except Exception:
                raise ValueError(
                    "[Rail='{}'] 'cTimeAvg' = '{}' invalid. Must be in the form vbct,vshct,vtct,avg".format(
                        name, str_cTA
                    )
                )
            try:
                cTimeAvg = (
                    CT_TBL[vbct] << 9
                    | CT_TBL[vshct] << 6
                    | CT_TBL[vtct] << 3
                    | AVG_TBL[avg]
                )

                # Also disable anything with cTime == 0 (T,Vshunt,Vbus)
                enableModes = (vtct > 0) << 2 | (vshct > 0) << 1 | (vbct > 0) << 0
            except KeyError:
                raise ValueError(
                    "Invalid cTimeAvg {},{},{},{}. Supported values for ctime {}, and for avg {} ".format(
                        vbct, vshct, vtct, avg, [*CT_TBL], [*AVG_TBL]
                    )
                )
        self.mode = None
        self.sub_rails = []
        self.name = name
        # if yaml file contains multiple mode separated by ','
        # recursively create sub_rails under eatch rail
        for mn in mode.split(","):
            if self.mode is None:
                try:
                    self.mode = ina_mode[mn.strip()]
                except Exception:
                    raise ValueError(
                        "[Rail='{}'] 'mode' = '{}' invalid. Must be {}".format(
                            name, mode, [*ina_mode]
                        )
                    )
                if self.ina != "GPIO" and self.mode == INA_MODE_GPIO:
                    raise ValueError(
                        "[Rail='{}'] 'mode' = '{}' invalid. Not a GPIO rail.".format(
                            name, mode
                        )
                    )
            else:
                if self.ina == "GPIO":
                    raise ValueError(
                        "[Rail='{}'] 'mode' = '{}' invalid for GPIO. Must be only one.".format(
                            name, mode
                        )
                    )

                self.sub_rails.append(
                    Rail(
                        self.name,
                        self.ina,
                        mn.strip(),
                        self.str_bus,
                        self.str_adr,
                        self.str_hz,
                        self.str_ohm,
                        self.str_maxA,
                        self.str_cTA,
                        self.subsystem,
                    )
                )
        self.rail_id = -1
        self.bus = bus
        self.adr = adr
        self.ohm = ohm
        self.maxA = maxA
        self.cTimeAvg = cTimeAvg
        self.enableModes = enableModes
        self.unitStr = ""

        if hz > 0:
            self.period_us = int(1000000.0 / hz)
            self.hz = hz
        else:
            self.period_us = 0

        self.value = 0.0  # Last received sample value, in physical units
        self.timestamp_absolute = None  # Sample MCU timestamp (usec, undef zero point)
        self.timestamp_delta = 0  # MCU time elapsed between samples (usec)

        self.received_time = None  # Host time @ sample received (sec, undef zero point)
        self.last_received_interval = 0  # Host time delta between received samples

        self.sample_count = 0  # Total number of samples received and processed

    def set_id(self, rail_id):
        self.rail_id = rail_id
        for s_rail_id, s_rail in enumerate(self.sub_rails):
            # For sub_rails use upper 2 bits as index
            if s_rail_id > MAX_S_RAIL:
                raise Exception("Invalid sub rail id : " + str(s_rail_id))
            s_rail.set_id(s_rail_id + 1 << 6 | rail_id)

    # Used to initially set the zero point timestamp (in microseconds) for the
    # board this rail belongs to and the system. board_zero is the timestamp of
    # the board when collection started. The board's clock wraps every 16.7s,
    # therefore the timestamps received by set_sample() are not absolute
    # timestamps. They are compared to the last timestamp received to determine
    # a delta, which is added to a running timestamp. board_zero is used to
    # determine the delta for the first arriving timestamp. sys_zero is the time
    # to consider the zero point that collection started.
    # For example, if sys_zero=10 and board_zero=3 then the first sample for
    # the rail arrives with timestamp=5, then the sample's timestamp will be 12.
    # The system's zero point may come from the sentinel board, host machine,
    # DUT, or some other arbitrary user-provided timestamp.
    # NOTE: The low 24 bits of the board_zero here are interpreted relative to
    # the 24-bit timestamp in incoming samples; if a sample comes in that has a
    # smaller timestamp value than these 24 bits, it will be assumed a timestamp
    # wraparound has occured and 16.7sec will be added. Therefore, these 24 bits
    # must match or be slightly behind the low 24 bits of the current board
    # timestamp (up to reasonable processing delays); you cannot set them
    # arbitrarily, or you will find spurious 16.7s offsets injected. Only the high
    # bits are freely configurable here (and it's best just to use the board's
    # timestamp for that to limit confusion). See set_sample() for the wraparound
    # recovery logic. If you want to adjust the timebase with some arbitrary zero
    # point, add that in separately in sys_zero.
    def set_timebase(self, board_zero, sys_zero):
        self.last_timestamp = board_zero
        self.timestamp_absolute = sys_zero
        for subrail in self.sub_rails:
            subrail.set_timebase(board_zero, sys_zero)

    def set_sample(self, timestamp, sample):
        self.timestamp_delta = (timestamp - self.last_timestamp) & 0xFFFFFF
        self.last_timestamp = timestamp
        # track full, un-truncated timestamp, as long as the delay between samples < 16.7s
        self.timestamp_absolute += self.timestamp_delta

        self.sample_count += 1

        new_received_time = time.perf_counter()
        if self.received_time is not None:
            self.last_received_interval = new_received_time - self.received_time
        self.received_time = new_received_time

        ina_228_values = {
            INA_MODE_CURRENT: ((sample >> 4) * self.Current_LSB),
            INA_MODE_POWER: (sample * 3.2 * self.Current_LSB),
            INA_MODE_ENERGY: (sample * 16 * 3.2 * self.Current_LSB),
            INA_MODE_CHARGE: (sample * self.Current_LSB),
            INA_MODE_VBUS: ((sample >> 4) * 195.3125 / 1000 / 1000),
            INA_MODE_VSHUNT: ((sample >> 4) * 312.5 / 1000 / 1000 / 1000),
            INA_MODE_DIETEMP: ((sample) * 7.8125 / 1000),
            INA_MODE_GPIO: (sample),
        }

        self.value = ina_228_values[self.mode]


def pack_rail_cfg_msg(rail_id, bus, adr, rdreg, rdlen, period):
    ############################################
    # MUST MATCH! from src/usb_sentinel.h
    ############################################
    # typedef struct {
    #    uint8_t msg_id;
    #    uint8_t rail_id;
    #    uint8_t bus;
    #    uint8_t adr;
    #    uint8_t rdreg;
    #    uint8_t rdlen;
    #    uint32_t period;
    # } sentinel_rail_cfg_msg_t;
    return struct.pack(
        "!BBBBBBI", SENTINEL_MSG_RAIL_CFG, rail_id, bus, adr, rdreg, rdlen, period
    )


def pack_i2c_write_read(bus, adr, rdlen, wrbuf):
    ############################################
    # MUST MATCH! from src/usb_sentinel.h
    ############################################
    # typedef struct {
    #    uint8_t msgid;
    #    uint8_t bus;
    #    uint8_t adr;
    #    uint8_t rdlen;
    #
    #    //wrlen is implied by how much data is in the buffer
    #    uint8_t wrdata[0];
    # } sentinel_i2c_write_read_msg_t;
    hdr = struct.pack("!BBBB", SENTINEL_MSG_I2C_WRITE_READ, bus, adr, rdlen)
    if wrbuf is None:
        return hdr
    return hdr + wrbuf


def pack_board_info_req():
    # board info req has no data, just the msgid
    return struct.pack("!B", SENTINEL_MSG_BOARD_INFO_REQ)


def get_rail(rails, rail_id):
    if rail_id & 0xC0:
        # Root is id 0 and first sub_rail_id start with 1
        rail = rails[rail_id & 0x3F].sub_rails[(rail_id >> 6) - 1]
    else:
        rail = rails[rail_id]
    return rail


def unpack_msg(buf, rails):

    ###########################################
    # MUST MATCH! from src/usb_sentinel.h
    ###########################################

    assert len(buf) <= 64

    if len(buf) < 1:
        # USB ZLP ignored
        return None

    (msgid, error) = struct.unpack("!BB", buf[:2])

    if msgid == SENTINEL_MSG_DATA_STREAM:
        #
        # typedef struct {
        #     uint8_t msgid;
        #     uint8_t error;
        #     uint8_t reserved[2];
        #     uint8_t data[0];
        # } sentinel_data_msg_t;
        #
        msg = {}
        msg["msgid"] = msgid
        msg["samples"] = []
        num_bytes = int(len(buf))
        idx = 4
        while idx < num_bytes:
            (id_ts,) = struct.unpack("!I", buf[idx : idx + 4])
            rail_id = id_ts >> 24
            timestamp = id_ts & 0xFFFFFF
            signbit = (buf[idx + 4] & 0x80) == 0x80
            rail = get_rail(rails, rail_id)
            sample = int.from_bytes(buf[idx + 4 : idx + 4 + rail.rdlen], "big")
            if signbit and rail.signed:
                sample = sample - (1 << (8 * rail.rdlen))
            msg["samples"].append((rail_id, timestamp, sample))
            idx += 4 + rail.rdlen
        msg["errors"] = []
        if error > 0:
            for i in range(len(SENTINEL_MSG_ERROR_TYPES)):
                if error >> i & 1:
                    msg["errors"].append(SENTINEL_MSG_ERROR_TYPES[i])
        return msg
    else:
        eprint("Bad message: " + str(len(buf)) + "B -- " + str(list(buf)))
        raise Exception("Unrecognized data message ID: " + str(msgid))
        # return None # <-- ignore bad message


def unpack_rsp(buf):

    ###########################################
    # MUST MATCH! from src/usb_sentinel.h
    ###########################################

    (msgid,) = struct.unpack("!B", buf[:1])

    if msgid == SENTINEL_MSG_BOARD_INFO_RSP:
        #
        # typedef struct {
        #     uint8_t  msgid;
        #     uint16_t fw_ver_major;
        #     uint16_t fw_ver_minor;
        #     uint32_t board_id;
        #     uint8_t  board_index;
        #     uint8_t  board_up_dn;
        #     uint64_t uptime;
        #     uint32_t err_cnt[3]; [i2c_err, buff_ovrrns, schd_misses]
        # } sentinel_board_info_msg_t;
        #

        if len(buf) < 5:
            raise Exception(
                "Bad board info response: " + str(len(buf)) + "B -- " + str(list(buf))
            )

        (major, minor, board_id) = struct.unpack("!HHI", buf[1:9])

        msg = {}
        msg["msgid"] = msgid
        msg["fw_ver_major"] = major
        msg["fw_ver_minor"] = minor
        msg["board_id"] = board_id
        msg["board_index"] = -1
        msg["board_up_dn"] = 0
        msg["uptime"] = 0
        msg["i2c_errors"] = 0
        msg["buff_overruns"] = 0
        msg["schd_missess"] = 0

        if len(buf) == 19:  # FW versions 0.6.0 - 0.7.1
            (
                board_up,
                board_dn,
                uptime,
            ) = struct.unpack("!BBQ", buf[9:19])
            msg["board_up_dn"] = (board_up << 5) | (board_dn << 4)
            msg["uptime"] = uptime
        elif len(buf) >= 31:  # FW v0.8.0+
            (
                board_index,
                boardupdn,
                uptime,
                i2c_err,
                buff_ovrrns,
                schd_misses,
            ) = struct.unpack("!BBQIII", buf[9:31])
            msg["board_index"] = board_index
            msg["board_up_dn"] = boardupdn
            msg["uptime"] = uptime
            msg["i2c_errors"] = i2c_err
            msg["buff_overruns"] = buff_ovrrns
            msg["schd_missess"] = schd_misses
        return msg

    if msgid == SENTINEL_MSG_RAIL_CFG_RSP:
        #
        # typedef struct {
        #    uint8_t msg_id;
        #    uint8_t rail_id;
        #    uint8_t success; // 1=success 0=failure
        # } sentinel_rail_cfg_rsp_t;
        #
        if len(buf) < 3:
            return None
        (msgid, rail_id, success) = struct.unpack("!BBB", buf[:3])
        msg = {}
        msg["msgid"] = msgid
        msg["rail_id"] = rail_id
        msg["success"] = success != 0
        return msg

    if msgid == SENTINEL_MSG_I2C_WRITE_READ_RSP:
        # typedef struct {
        #    uint8_t msgid;
        #    uint8_t success; // 1=success 0=failure
        #    uint8_t rdlen;
        #    uint8_t reserved;
        #    uint8_t rddata[0];
        # } sentinel_i2c_write_read_rsp_t;
        expected_rdlen = len(buf) - 4
        (msgid, success, rdlen, reserved, rddata) = struct.unpack(
            "!BBBB{}s".format(expected_rdlen), buf
        )
        if success is None or success == 0:
            return None
        if rdlen != expected_rdlen or len(rddata) != rdlen:
            return None
        return rddata

    if msgid == SENTINEL_MSG_BOARD_INFO_RSP:
        #
        # typedef struct {
        #    uint8_t msg_id;
        #    uint8_t success; // 1=success 0=failure
        # } sentinel_enter_bootloader_rsp_t;
        #
        if len(buf) < 2:
            return None
        (msgid, success) = struct.unpack("!BB", buf[:2])
        msg = {}
        msg["msgid"] = msgid
        msg["success"] = success != 0
        return msg

    eprint("Bad message: " + str(len(buf)) + "B -- " + str(list(buf)))
    raise Exception("Unrecognized reponse message ID: " + str(msgid))
    # return None # <-- ignore bad message


def clear_screen():
    os.system("cls" if os.name == "nt" else "clear")


def print_row(rails, verbose):
    print("|", end="")

    # print cells with 'name'
    for i in range(0, min(NUM_COLUMNS, len(rails))):
        r = rails[i]
        fmt = style.BOLD + "{:^" + str(MAX_NAME_PRINT_LEN) + "s}" + style.END + "|"
        print(fmt.format(r.name[:MAX_NAME_PRINT_LEN]), end="")

    # print empty cells if we don't have enough
    for _i in range(0, NUM_COLUMNS - len(rails)):
        fmt = "{:^" + str(MAX_NAME_PRINT_LEN) + "s}|"
        print(fmt.format(""), end="")
    print("")

    num_Rows = 0
    for i in range(0, min(NUM_COLUMNS, len(rails))):
        if len(rails[i].sub_rails) > num_Rows:
            num_Rows = len(rails[i].sub_rails)

    for rws in range(num_Rows + 1):
        print("|", end="")
        # print cells with 'value'
        for i in range(0, min(NUM_COLUMNS, len(rails))):
            if rws > 0:
                if rws > len(rails[i].sub_rails):
                    fmt = "{:^" + str(MAX_NAME_PRINT_LEN) + "s}|"
                    print(fmt.format(""), end="")
                    continue
                r = rails[i].sub_rails[rws - 1]
            else:
                r = rails[i]
            val_fmt = "{:." + str(NUM_DECIMAL_PLACES) + "f}{}"
            if r.unitStr is not None and len(r.unitStr) > 0:
                val_str = val_fmt.format(r.value, " " + r.unitStr)
            else:
                val_str = val_fmt.format(r.value, "")
            fmt = "{:^" + str(MAX_NAME_PRINT_LEN) + "s}|"
            print(fmt.format(val_str), end="")

        # print empty cells if we don't have enough
        for _i in range(0, NUM_COLUMNS - len(rails)):
            fmt = "{:^" + str(MAX_NAME_PRINT_LEN) + "s}|"
            print(fmt.format(""), end="")
        print("")

    if verbose:
        print("|", end="")

        # print cells with 'sample count'
        for i in range(0, min(NUM_COLUMNS, len(rails))):
            r = rails[i]
            samps_fmt = "{:.0f} {:.4f}s {:d}us"
            samps_str = samps_fmt.format(
                r.sample_count, r.last_received_interval, r.timestamp_delta
            )
            fmt = "{:^" + str(MAX_NAME_PRINT_LEN) + "s}|"
            print(fmt.format(samps_str), end="")

        # print empty cells if we don't have enough
        for _i in range(0, NUM_COLUMNS - len(rails)):
            fmt = "{:^" + str(MAX_NAME_PRINT_LEN) + "s}|"
            print(fmt.format(""), end="")
        print("")


def update_display(config_name, board_id, ver_maj, ver_min, rails, verbose=False):
    clear_screen()

    for msg in static_msgs:
        print(msg)
    for board in sorted(board_errors.keys()):
        print("Board #" + str(board) + " errors: " + board_errors[board])

    table_width = MAX_NAME_PRINT_LEN * NUM_COLUMNS + NUM_COLUMNS + 1

    print("{}".format(table_width * "-"))
    fmt = (
        "| "
        + style.BOLD
        + "{:<20}{:^"
        + str(table_width - 44)
        + "}{:>20}"
        + style.END
        + " |"
    )
    print(
        fmt.format(
            "Board: " + board_names[board_id],
            config_name,
            "FW: v" + str(ver_maj) + "." + str(ver_min),
        )
    )
    print("{}".format(table_width * "-"))

    for i in range(0, len(rails), NUM_COLUMNS):
        print_row(rails[i : i + NUM_COLUMNS], verbose)
        print("{}".format(table_width * "-"))


def sentinel_INA_test_config(test, sample_rate):
    board_name = "SENTINEL_INA228_2"
    ina = "INA228"
    mode = test.upper()
    if sample_rate:
        hz = int(sample_rate)
    else:
        hz = 3
    ohm = 0.1
    maxA = 0.04096 / ohm
    cTimeAvg = "50,50,50,1"
    rails = []
    board_id = SENTINEL_BOARD_STANDALONE

    for bus in range(1, 4):
        for addr in range(0x40, 0x50):
            rails.append(
                Rail(
                    "B_" + str(bus) + "_" + hex(addr),
                    ina,
                    mode,
                    str(bus),
                    str(addr),
                    str(hz),
                    str(ohm),
                    str(maxA),
                    cTimeAvg,
                )
            )

    boards = [rails]
    return board_name, board_id, boards, None


def load_config(configfile, config):
    if type(configfile) == str:
        with open(configfile, "r") as f:
            y = yaml.load(f, Loader=yaml.SafeLoader)
    else:
        y = configfile  # If we passed in a config dict it shouldn't be parsed

    if "board_name" not in y:
        raise ValueError("'board_name' required in config file")

    if "board_id" in y:
        board_id = y["board_id"]
    else:
        board_id = SENTINEL_BOARD_STANDALONE

    board_name = y["board_name"]

    if "boards" not in y:
        boards = [[]]
        y_b = range(1)
    else:
        boards = [[] for i in range(MAX_BOARDS)]
        y_b = y["boards"]

    for board in y_b:
        if "boards" not in y:
            y_r = y["rails"]
        else:
            y_r = y["boards"][board]["rails"]
        for name in y_r:
            rail = y_r[name]
            ina = None
            mode = None
            bus = None
            adr = None
            ohm = None
            maxA = None
            cTimeAvg = None
            subsys = None
            parent = None

            if "ina" in rail:
                ina = rail["ina"]

            if config.sampling_mode:
                mode = config.sampling_mode.upper()
            elif "mode" in rail:
                mode = rail["mode"]

            if "bus" in rail:
                bus = rail["bus"]

            if "adr" in rail:
                adr = rail["adr"]
            elif "ina_address" in rail:
                adr = rail["ina_address"]

            if "ohm" in rail:
                ohm = rail["ohm"]
            elif "design_resistance" in rail:
                ohm = rail["design_resistance"]

            if config.sample_rate:
                hz = int(config.sample_rate)
            elif "hz" in rail:
                hz = rail["hz"]

            if "maxA" in rail:
                maxA = rail["maxA"]

            if "cTimeAvg" in rail:
                cTimeAvg = rail["cTimeAvg"]

            if "subsys" in rail:
                subsys = rail["subsys"]

            if "parent" in rail:
                parent = rail["parent"]

            if config.filter_rails and not re.search(
                config.filter_rails, yaml.dump({name: rail})
            ):

                # Filtered out; don't include this rail
                continue

            boards[board].append(
                Rail(name, ina, mode, bus, adr, hz, ohm, maxA, cTimeAvg, subsys, parent)
            )

    timesync_trigger_config = None
    if config.timebase == "trigger":
        if "timesync_trigger" not in y:
            raise ValueError(
                "--timebase=trigger requires a 'timesync_trigger' to be "
                "defined in config file"
            )
        if (
            "mode" not in y["timesync_trigger"]
            or y["timesync_trigger"]["mode"] not in ina_mode
        ):
            raise ValueError("'timesync_triggger' must define a valid mode")
        if "rail" not in y["timesync_trigger"]:
            raise ValueError("'timesync_triggger' must define a rail")
        if "threshold" not in y["timesync_trigger"]:
            raise ValueError("'timesync_trigger' must define a threshold")

        timesync_trigger_config = y["timesync_trigger"]
        if "board" not in y["timesync_trigger"]:
            timesync_trigger_config["board"] = 0

    return board_name, board_id, boards, timesync_trigger_config


def open_socket(addr, port):
    # Open TCP/IP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((addr, port))
    return sock


def sentinel_cmd(usbdev, buf, rsp_expected=True):
    global num_hops_to_dest_board
    # Recursively add head and tail to the message to tunnel
    # it through UARTs
    for _i in range(num_hops_to_dest_board):
        hdr = struct.pack("!BB", SENTINEL_MSG_USB_TUN_START, len(buf))
        tale = struct.pack("!B", SENTINEL_MSG_MAX)
        buf = hdr + buf + tale
    usbdev.write(USB_TO_DEV_CFG_EP, buf)
    if rsp_expected:
        msg = usbdev.read(USB_FROM_DEV_CFG_EP, 0xFFFF)
        return unpack_rsp(msg)
    return True


def sentinel_get_board_info(usbdev):
    rsp = sentinel_cmd(usbdev, pack_board_info_req())
    if rsp is None or rsp["msgid"] != SENTINEL_MSG_BOARD_INFO_RSP:
        return None
    return rsp


def sentinel_enter_bootloader(usbdev):
    rsp = sentinel_cmd(usbdev, struct.pack("!B", SENTINEL_MSG_ENTER_BOOTLOADER))
    if rsp is None or rsp["msgid"] != SENTINEL_MSG_ENTER_BOOTLOADER_RSP:
        return None
    return rsp


def sentinel_rail_r8(
    usbdev,
    rail,
    reg_adr,
):
    cmd = pack_i2c_write_read(rail.bus, rail.adr, 1, bytes([reg_adr]))
    rsp = sentinel_cmd(usbdev, cmd)
    if isinstance(rsp, bytes) and len(rsp) == 1:
        return struct.unpack("!B", rsp)[0]
    return None


#################################################
# Example of using generic register read/write
#################################################
# if rail.ina == INA228:
#    mfgid = sentinel_rail_r16(usbdev, rail, 0x3E)
#    devid = sentinel_rail_r16(usbdev, rail, 0x3F)
#    if mfgid is not None and devid is not None:
#        print("[Rail='{}'] mfgID=0x{:04X} devID=0x{:04X}".format(rail.name, mfgid, devid))
#
#    read initial value
#    shunt_cal = sentinel_rail_r16(usbdev, rail, 2)
#    if shunt_cal is not None:
#        print("[Rail='{}'] shunt_cal before = {:04X}".format(rail.name, shunt_cal))
#
#    overwrite the value
#    sentinel_rail_w16(usbdev, rail, 2, 0x1234)
#
#    read back the changed value
#    shunt_cal = sentinel_rail_r16(usbdev, rail, 2)
#    if shunt_cal is not None:
#        print("[Rail='{}'] shunt_cal after  = {:04X}".format(rail.name, shunt_cal))
def sentinel_rail_r16(usbdev, rail, reg_adr):
    cmd = pack_i2c_write_read(rail.bus, rail.adr, 2, bytes([reg_adr]))
    rsp = sentinel_cmd(usbdev, cmd)
    if isinstance(rsp, bytes) and len(rsp) == 2:
        return struct.unpack("!H", rsp)[0]
    return None


def sentinel_rail_r24(usbdev, rail, reg_adr):
    cmd = pack_i2c_write_read(rail.bus, rail.adr, 3, bytes([reg_adr]))
    rsp = sentinel_cmd(usbdev, cmd)
    if isinstance(rsp, bytes) and len(rsp) == 3:
        (a, b, c) = struct.unpack("!BBB", rsp)
        return (a << 16) | (b << 8) | c
    return None


def sentinel_rail_r32(usbdev, rail, reg_adr):
    cmd = pack_i2c_write_read(rail.bus, rail.adr, 4, bytes([reg_adr]))
    rsp = sentinel_cmd(usbdev, cmd)
    if isinstance(rsp, bytes) and len(rsp) == 3:
        return struct.unpack("!I", rsp)[0]
    return None


def sentinel_rail_w8(usbdev, rail, reg_adr, val):
    if val < 0 or val > 2**8:
        eprint("Warning: " + rail.name + " has out-of-range config value " + str(val))
    cmd = pack_i2c_write_read(rail.bus, rail.adr, 0, struct.pack("!BB", reg_adr, val))
    return sentinel_cmd(usbdev, cmd)
    # TODO check success response


def sentinel_rail_w16(usbdev, rail, reg_adr, val):
    if val < 0 or val > 2**16:
        eprint("Warning: " + rail.name + " has out-of-range config value " + str(val))
    cmd = pack_i2c_write_read(rail.bus, rail.adr, 0, struct.pack("!BH", reg_adr, val))
    return sentinel_cmd(usbdev, cmd)
    # TODO check success response


def sentinel_rail_w32(usbdev, rail, reg_adr, val):
    if val < 0 or val > 2**32:
        eprint("Warning: " + rail.name + " has out-of-range config value " + str(val))
    cmd = pack_i2c_write_read(rail.bus, rail.adr, 0, struct.pack("!BI", reg_adr, val))
    return sentinel_cmd(usbdev, cmd)
    # TODO check success response


def sentinel_reboot(usbdev):
    return sentinel_cmd(
        usbdev, struct.pack("!B", SENTINEL_MSG_REBOOT), rsp_expected=False
    )


def sentinel_cfg_reset(usbdev):
    return sentinel_cmd(
        usbdev, struct.pack("!B", SENTINEL_MSG_CFG_RESET), rsp_expected=False
    )


def sentinel_streaming_on_off(usbdev, on_off):
    global streaming_on
    streaming_on = on_off
    return sentinel_cmd(
        usbdev,
        struct.pack("!BB", SENTINEL_MSG_STREAMING_ON_OFF, on_off),
        rsp_expected=False,
    )


def sentinel_INA_reset(usbdev, bus=ALL_BUSSES):
    return sentinel_cmd(
        usbdev,
        struct.pack("!BB", SENTINEL_MSG_INA_RESET, bus),
        rsp_expected=False,
    )


def sentinel_INA_test(usbdev, test_on=True):
    return sentinel_cmd(
        usbdev,
        struct.pack("!BB", SENTINEL_MSG_INA_TEST, test_on),
        rsp_expected=False,
    )


def sentinel_set_timestamp(usbdev, timestamp):
    return sentinel_cmd(
        usbdev,
        struct.pack("!BQ", SENTINEL_MSG_SET_TIMESTAMP, timestamp),
        rsp_expected=False,
    )


def sentinel_sync_timestamp(orderusbdev):
    host_t0 = time.time()
    timestamps = []
    for dev in orderusbdev:
        board_info = sentinel_get_board_info(dev)
        timestamps.append(board_info["uptime"])
    max_ts = max(timestamps)
    min_ts = min(timestamps)
    # if time drift is more than 1 msec sync the timestamp.
    if (max_ts - min_ts) > 1000:
        for dev in orderusbdev:
            # Setting to max_ts (going back in time can cause paradox).
            # Include an offset for the amount of time it's taken to get here,
            # particularly since we can't set all the boards simultaneously.
            offset = int((time.time() - host_t0) * 1000000)
            sentinel_set_timestamp(dev, max_ts + offset)


def get_sentinel_timestamp(orderusbdev):
    # TODO this function name is misleading given recent changes. Can we fix it to
    # to return the whole timestamp rather than the low 24 bits?
    dev = orderusbdev[0]
    board_info = sentinel_get_board_info(dev)
    sent_time = int(board_info["uptime"])
    return sent_time


def is_prerelease_fw(board_info):
    return board_info["fw_ver_major"] == 0 and board_info["fw_ver_minor"] == 0


def check_board_version_at_least(board_info, min_major, min_minor):
    return is_prerelease_fw(board_info) or (
        board_info["fw_ver_major"] > min_major
        or (
            board_info["fw_ver_major"] == min_major
            and board_info["fw_ver_minor"] >= min_minor
        )
    )


# The board_index attribute is not available before FW v0.8, but the up-down
# daisy-chain status can be used instead to determine ordering.
def get_daisy_chain_order_legacy(allusbdev):
    if len(allusbdev) > 3:
        raise ValueError(
            "Board FW prior to v0.8 is not compatible in daisy chain longer than 3."
            " Please update all boards."
        )

    boards = [None for k in range(len(allusbdev))]
    for dev in allusbdev:
        try:
            board_info = sentinel_get_board_info(dev)
        except usb.core.USBError as e:
            eprint("Warning: could not query board (possibly busy?) -- " + str(e))
            dev.finalize()
            continue
        if board_info is None:
            raise ValueError("Failed to get valid Sentinel board info")

        if board_info["board_up_dn"] == 0x30:
            # Both up and down bits are set, board must be between 2 others.
            boards[1] = dev
        elif board_info["board_up_dn"] == 0x20:
            # Only up bit is set, this must be the first board.
            boards[0] = dev
        elif board_info["board_up_dn"] == 0x10:
            # Only down bit is set, this must be the last board.
            boards[-1] = dev
        elif board_info["board_up_dn"] == 0:
            # Solo board.
            boards[0] = dev

    return boards


def get_daisy_chain_order(allusbdev):
    # Retrieve the daisy-chain location information from each board and index in order
    boards = [None for k in range(len(allusbdev))]

    use_legacy_ordering = False
    for dev in allusbdev:
        try:
            board_info = sentinel_get_board_info(dev)
        except usb.core.USBError as e:
            eprint("Warning: could not query board (possibly busy?) -- " + str(e))
            dev.finalize()
            continue
        if board_info is None:
            raise ValueError("Failed to get valid Sentinel board info")

        if len(allusbdev) > 1 and not check_board_version_at_least(
            board_info, MIN_FW_VER_MAJOR, MIN_FW_DAISY_CHAIN_VER_MINOR
        ):
            raise ValueError(
                "Board FW v{}.{} is not compatible in daisy chain. "
                "Please update all boards to at least FW v{}.{}".format(
                    board_info["fw_ver_major"],
                    board_info["fw_ver_minor"],
                    MIN_FW_VER_MAJOR,
                    MIN_FW_DAISY_CHAIN_VER_MINOR,
                )
            )

        board_index = board_info["board_index"]
        if board_index < 0:
            use_legacy_ordering = True
            continue

        while board_index >= len(boards):
            eprint("Warning: daisy chain implies a board not detected via USB")
            boards += [None]

        if boards[board_index]:
            # We only support a single daisy chain at a time
            eprint("Warning: multiple boards found at chain index " + str(board_index))

        boards[board_index] = dev

    if use_legacy_ordering:
        # If one or more board's lacked a board_index use the up-down
        # daisy-chain status instead to determine the ordering.
        return get_daisy_chain_order_legacy(allusbdev)

    return boards


def sentinel_cfg_rail(usbdev, rail, sentinel_write_cfg=True):
    # Conversion Time or ctime, set in the ADC register of INA228
    # ctime = {0, 1, 2, 3, 4, 5, 6, 7}  correspnds to
    # Conversion time = {50, 84, 150, 280, 540, 1052, 2074, 4120) usec

    # Conversion times for bus voltage, shunt voltage and temeperature,
    # and number of individual conversions to average before reporting:
    ADC_RATES = {
        # (vbus + vshu + temp us) x avg   Refer INA228 data sheet for ENOB
        1: "6764",  # (2074 + 4120 + 2074 us) x 128 = 1058304 us => 0.945  Hz
        3: "5544",  # (1052 + 1052 + 540 us)  x 128 = 338432 us => 2.950   Hz
        4: "3425",  # ( 280 +  540 +  150 us) x 256 = 248320 us => 4.027   Hz
        10: "5752",  # (1052 + 4120 + 1052 us) x  16 = 99584  us => 10.042  Hz
        20: "4642",  # ( 540 + 2074 +  540 us) x  16 = 50464  us => 19.816  Hz
        30: "7721",  # (4120 + 4120 +  150 us) x   4 = 33560  us => 29.797  Hz
        50: "4422",  # ( 540 +  540 +  150 us) x  16 = 19680  us => 50.813  Hz
        88: "3322",  # ( 280 +  280 +  150 us) x  16 = 11360  us => 88.028  Hz
        100: "3621",  # ( 280 + 2074 + 150 us) x  4 = 10016  us => 99.840  Hz
        300: "3331",  # ( 280 + 280 +  280 us) x  4 = 3360   us => 297.619 Hz
        350: "3321",  # ( 280 + 280 +  150 us) x  4 = 2840   us => 352.113 Hz
        430: "2321",  # ( 150 + 280 +  150 us) x  4 = 2320   us => 431.035 Hz
        530: "4521",  # ( 540 + 1052 + 280 us) x  1 = 1872   us => 534.188 Hz
        1030: "3420",  # ( 280 +  540 + 150 us) x  1 = 970    us => 1030.93 Hz
        2600: "2210",  # ( 150 +  150 + 84 us)  x  1 = 384    us => 2604.16 Hz
        3140: "1210",  # (  84 +  150 + 84 us)  x  1 = 318    us => 3144.65 Hz
        3970: "1110",  # (  84 +  84 +  84 us)  x  1 = 252    us => 3968.25 Hz
        4590: "1100",  # (  84 +  84 +  50 us)  x  1 = 218    us => 4587.16 Hz
        5430: "0100",  # (  50 +  84 +  50 us)  x  1 = 184    us => 5434.78 Hz
        6670: "0000",  # (  50 +  50 +  50 us)  x  1 = 150    us => 6666.67 Hz
    }

    if rail.cTimeAvg is None:
        # Initialize to bottom of the table incase for loop does not find
        VBUSCT, VSHCT, VTCT, AVG = map(int, ADC_RATES[max(ADC_RATES)])
        for hz in sorted(ADC_RATES):
            if hz >= rail.hz:
                VBUSCT, VSHCT, VTCT, AVG = map(int, ADC_RATES[hz])
                break

        CTIME_AVG = VBUSCT << 9 | VSHCT << 6 | VTCT << 3 | AVG
    else:
        CTIME_AVG = rail.cTimeAvg

    if rail.enableModes is None:
        # Enable all (T,Vshunt,Vbus)
        ADC_MODES = 0x7
    else:
        ADC_MODES = rail.enableModes

    if rail.maxA is None:
        if rail.ohm > 0.0:
            rail.maxA = 0.04096 / rail.ohm
        else:
            rail.maxA = 10.0

    ina_228_reg = {
        INA_MODE_CURRENT: ("A", 0x07, 3, True, (rail.maxA / (2**19)), 2),
        INA_MODE_POWER: ("W", 0x08, 3, False, (rail.maxA / (2**19)), 2),
        INA_MODE_ENERGY: ("J", 0x09, 5, False, (rail.maxA / (2**19)), 2),
        INA_MODE_CHARGE: ("C", 0x0A, 5, True, (rail.maxA / (2**19)), 2),
        INA_MODE_VBUS: ("V", 0x05, 3, True, (rail.maxA / (2**19)), 255),
        INA_MODE_VSHUNT: ("V", 0x04, 3, True, (rail.maxA / (2**19)), 255),
        INA_MODE_DIETEMP: ("C", 0x06, 2, True, (rail.maxA / (2**19)), 255),
        INA_MODE_GPIO: (" ", 0x00, 1, False, 1, 255),
    }

    (
        rail.unitStr,
        rdreg,
        rail.rdlen,
        rail.signed,
        rail.Current_LSB,
        shuntCalReg,
    ) = ina_228_reg[rail.mode]

    if rail.ina != "GPIO":
        adc_cfg = 1 << 15 | ADC_MODES << 12 | CTIME_AVG
        if sentinel_write_cfg:
            sentinel_rail_w16(usbdev, rail, 1, adc_cfg)
        rail.SHUNT_CAL = int(13107.2 * (10**6) * rail.Current_LSB * rail.ohm)
        if sentinel_write_cfg:
            if shuntCalReg < 255:
                sentinel_rail_w16(usbdev, rail, shuntCalReg, rail.SHUNT_CAL)

    buf = pack_rail_cfg_msg(
        rail.rail_id, rail.bus, rail.adr, rdreg, rail.rdlen, rail.period_us
    )
    if sentinel_write_cfg:
        rsp = sentinel_cmd(usbdev, buf)
        if rsp is None or rsp["msgid"] != SENTINEL_MSG_RAIL_CFG_RSP:
            raise ValueError(
                "Incorrect msgid in response setting rail config: {}".format(
                    rsp["msgid"]
                )
            )

        if rsp["rail_id"] != rail.rail_id or not rsp["success"]:
            raise ValueError(
                "Failed to set rail config for rail '{}'".format(rail.name)
            )


class SampleConsumer:
    def __init__(self, rails):
        self.rails = rails
        self.samples = {}
        self.times = {}

        for rail in self.rails:
            self.samples[rail] = []
            self.times[rail] = []

        self.index_rail = rails[0]

        self.lock = threading.Lock()

    def update_rail(self, rail):
        # Consume a new sample. Note: This is called from a worker thread and not from the main thread
        with self.lock:
            self.samples[rail].append(rail.value)
            self.times[rail].append(rail.timestamp_absolute)

            if rail == self.index_rail:
                self.index_updated()

    def index_updated(self):
        # The "index rail" has been updated with a new sample -- a notification for subclasses to use.
        # Note: This is called from a worker thread and not from the main thread, with self.lock held!
        pass

    def complete(self):
        # Clean up consumer activity and report any final results, because the stream has ended.
        # Note: This is called from the main thread, but after all worker threads have terminated.
        pass


class TimeNormalizer(SampleConsumer):
    def __init__(self, rails):
        super().__init__(rails)

    def oldest_new_sample_time(self):
        if min([len(times) for times in self.times.values()]) == 0:
            return -1

        return min([times[-1] for times in self.times.values()])

    def next_index_time(self):
        return self.times[self.index_rail][0]

    def increment_index(self):
        self.times[self.index_rail] = self.times[self.index_rail][1:]
        self.samples[self.index_rail] = self.samples[self.index_rail][1:]

    def index_updated(self):
        while (
            self.times[self.index_rail]
            and self.oldest_new_sample_time() >= self.next_index_time()
        ):
            index_time = self.next_index_time()
            row = {}
            for rail in self.rails:
                assert len(self.times[rail]) == len(self.samples[rail])

                # terribly inefficient and don't care! :p
                while (
                    self.times[rail][0] < index_time
                    and self.times[rail][1] < index_time
                ):
                    self.times[rail] = self.times[rail][1:]
                    self.samples[rail] = self.samples[rail][1:]

                if self.times[rail][0] >= index_time:
                    row[rail] = self.samples[rail][0]
                else:
                    # linearly interpolate
                    span = self.times[rail][1] - self.times[rail][0]
                    alpha = (index_time - self.times[rail][0]) / span
                    assert alpha >= 0.0 and alpha <= 1.0
                    row[rail] = self.samples[rail][1] * alpha + self.samples[rail][
                        0
                    ] * (1 - alpha)
                    if rail.mode == INA_MODE_GPIO:
                        row[rail] = round(row[rail])

            self.update_row(index_time, row)
            self.increment_index()

    def update_row(self, index_time, row):
        pass


class NormalizedTimeCsvDumper(TimeNormalizer):
    def __init__(self, rails, file=sys.stdout, heading=True):
        super().__init__(rails)
        self.file = file

        if heading:
            row_csv = "time_us"
            for rail in self.rails:
                row_csv += (
                    ", " + rail.name + "(" + list(ina_mode.keys())[rail.mode] + ")"
                )
            print(row_csv, file=self.file)

    def update_row(self, index_time, row):
        row_csv = str(index_time)
        for rail in self.rails:
            row_csv += ", " + str(row[rail])
        print(row_csv, file=self.file)
        pass


class NormalizedTimeCsvTcpIp(TimeNormalizer):
    def __init__(self, rails, sock, heading=True):
        super().__init__(rails)
        self.sock = sock

        if heading:
            row_csv = "Time (" + rails[0].sampleRate + "Hz)"
            for rail in self.rails:
                row_csv += (
                    ", " + rail.name + "(" + list(ina_mode.keys())[rail.mode] + ")"
                )
            # Send metadata to TCP server
            message = "Sentinel Metadata >> " + row_csv
            message_size = len(message)
            self.sock.send(chr(message_size).encode("utf-32-be"))
            self.sock.send(message.encode())

    def update_row(self, index_time, row):
        row_csv = str(index_time)
        for rail in self.rails:
            row_csv += ", " + str(row[rail])
        # Send data to TCP server
        message = "Sentinel Plot Data >> " + row_csv
        message_size = len(message)
        self.sock.send(chr(message_size).encode("utf-32-be"))
        self.sock.send(message.encode())
        pass


class TestDataAnalyzer(SampleConsumer):
    def __init__(self, rails, test, usbdev, hz=3):
        super().__init__(rails)
        self.test = test.upper()
        self.test_start_time = time.time()
        self.test_duration = 0
        self.usbdev = usbdev
        self.test_passed = True
        if hz is None:
            self.hz = 3.0
        else:
            self.hz = float(hz)
        sentinel_INA_test(usbdev)

    def update_rail(self, rail):
        super().update_rail(rail)

        # Run the test for 5 sec.
        if (time.time() - self.test_start_time) < 5.0:
            return

        count_set = set()
        ts_set = set()
        sample_data_set = set()
        self.test_duration = time.time() - self.test_start_time

        # Verify that sample_count in all rails converge to same value with in 5.1 sec.
        for r in range(0, len(self.rails)):
            count_set.add(self.rails[r].sample_count)
            ts_set.add(self.rails[r].timestamp_delta)
            sample_data_set.add(round(self.rails[r].value, 5))

        # Collect few more samples if rail sample count is unequal
        if len(count_set) > 1:
            if self.test_duration < 5.1:
                return

        final_board_info = sentinel_get_board_info(self.usbdev)

        # Stop the data streaming and UI updates (asynchronously, alas)
        sentinel_INA_test(self.usbdev, test_on=False)
        global is_exiting
        is_exiting = True  # would prefer end_all_read_threads but don't have the arg

        if (max(count_set) - min(count_set)) > 3:
            eprint(f"{self.test} TEST FAILED. Large rail sample count mismatch.")
            self.test_passed = False

        min_s = min(sample_data_set)
        max_s = max(sample_data_set)
        t_dev = max_s - min_s

        # If all rails show same temperature most likely it is read failure and
        # need investigation.
        if max_s == min_s:
            eprint(f"{self.test} TEST FAILED. INA read failure (identical values).")
            self.test_passed = False

        if self.test == "DIETEMP":
            # If the temperature variation is too large some thing may be wrong.
            if (t_dev) > 5.0:
                eprint(f"{self.test} TEST FAILED. Large temperature variation.")
                self.test_passed = False

        # Verify that sample rate deviation is with in the range (less than 5%)
        min_ts = min(ts_set)
        max_ts = max(ts_set)
        t_dev = max_ts - min_ts
        # "Timestamp" here is sample-to-sample interval in micro sec
        mean_sampling = 2000000.0 / (min_ts + max_ts)
        if abs(mean_sampling - self.hz) / self.hz > 0.05:
            eprint(
                f"{self.test} TEST FAILED. Rail sampling rate deviation greater than 5%"
            )
            self.test_passed = False

        if self.test_passed:
            eprint(f"{self.test} TEST PASSED.")

        eprint(f"Test duration {round(self.test_duration, 4)} sec.")
        eprint(f"Rail Sampling count variation {count_set}")
        eprint(
            f"Rail Sampling rate deviation {round(1000000.0/max_ts,2)}  -  {round(1000000.0/min_ts,2)} Average sampling rate {round(mean_sampling,1)} Hz"
        )
        if self.test == "DIETEMP":
            eprint(f"Temperature variation {min_s} to {max_s} Deg C.")
        eprint(final_board_info)
        exit(self.test_passed)


class LiveSampleScope(SampleConsumer):
    def __init__(
        self, rails, max_history=200, max_rails=100, update_period=2.0, ymax=None
    ):
        super().__init__(rails)

        self.last_update_time = 0

        self.max_rails_plotted = max_rails
        self.max_history = max_history
        self.update_period = update_period
        self.ymax = ymax

        import matplotlib.pyplot as plt

        self.plt = plt
        self.fig = plt.figure()
        self.fig.canvas.manager.set_window_title("Sentinel Live Scope")
        self.fig.show()
        self.ready_to_redraw = False

    def trim_history(self, rail, trim_length):
        # Call with self.lock held
        if len(self.times[rail]) > trim_length:
            self.times[rail] = self.times[rail][-trim_length:]
            self.samples[rail] = self.samples[rail][-trim_length:]

    def index_updated(self):
        now = time.time()
        if now - self.last_update_time > self.update_period:
            # (plot only infrequently because expensive...)
            for rail in self.rails:
                self.trim_history(rail, self.max_history)

            self.ready_to_redraw = True

    def redraw(self):
        # Note: This should be called only from the main thread, because matplotlib
        # is not thread-safe
        self.fig.clear()

        for rail in self.rails[: self.max_rails_plotted]:
            with self.lock:
                # Grab these atomically -- they're the only mutating variables we're looking
                # at, and we want them to be the same length and not half-way trimmed.
                times = self.times[rail].copy()
                samples = self.samples[rail].copy()

            if rail.rail_id >= 30:
                linestyle = ":"
            elif rail.rail_id >= 20:
                linestyle = "-."
            elif rail.rail_id >= 10:
                linestyle = "--"
            else:
                linestyle = None

            if self.ymax is not None:
                self.plt.ylim(top=float(self.ymax))
            self.plt.plot(
                times,
                samples,
                label=(rail.name + "(" + list(ina_mode.keys())[rail.mode] + ")"),
                linestyle=linestyle,
            )

        self.plt.legend(loc="lower left")
        self.fig.canvas.draw_idle()
        self.plt.pause(0.0001)

        self.last_update_time = time.time()
        self.ready_to_redraw = False


class SummarizingDataAnalyzer(SampleConsumer):
    def __init__(self, rails, usbdev, length_seconds, printer):
        super().__init__(rails)
        self.usbdev = usbdev
        self.length_seconds = length_seconds
        self.start_time = time.time()
        self.printer = printer

    def calculate_stats(self, samples):
        total = 0
        minimum = 999999999
        maximum = -999999999

        for sample in samples:
            total += sample
            minimum = min(minimum, sample)
            maximum = max(maximum, sample)
        avg = (total * 1.0) / (len(samples) * 1.0)

        avg = (total * 1.0) / (len(samples) * 1.0)
        return (avg, minimum, maximum)

    def complete(self):
        clear_screen()

        rail_stats = {
            rail: self.calculate_stats(samples)
            for (rail, samples) in self.samples.items()
        }
        self.printer.print(rail_stats)


class RailStatsPrinter:
    def __init__(self):
        pass

    def print(self, rail_stats):
        pass


class StandardRailPrinter(RailStatsPrinter):
    def __init__(self, rails):
        # Assume all rails use the same units.
        self.unit = rails[0].unitStr
        self.longest_subsys_name = 15
        self.longest_rail_name = 26
        for rail in rails:
            subsyslen = len(rail.subsystem) if rail.subsystem else 0
            self.longest_subsys_name = max(self.longest_subsys_name, subsyslen)
            self.longest_rail_name = max(self.longest_rail_name, len(rail.name))

    def print(self, rail_stats):
        fmt = (
            style.BOLD
            + "{:^"
            + str(self.longest_subsys_name)
            + "s} {:^"
            + str(self.longest_rail_name)
            + "s} {:^15s} {:^15s} {:^15s}"
            + style.END
        )
        print(
            fmt.format(
                "Subsystem",
                "Rail",
                "Average (" + self.unit + ")",
                "Min (" + self.unit + ")",
                "Max (" + self.unit + ")",
            )
        )
        total_avg = 0
        fmt = (
            "{subsys:<"
            + str(self.longest_subsys_name)
            + "s} {rail:<"
            + str(self.longest_rail_name)
            + "s} {avg:^15f} {min:^15f} {max:^15f}"
        )
        for (rail, stats) in rail_stats.items():
            print(
                fmt.format(
                    subsys=rail.subsystem if rail.subsystem else "",
                    rail=rail.name,
                    avg=round(stats[0], 4),
                    min=round(stats[1], 4),
                    max=round(stats[2], 4),
                )
            )
            total_avg += stats[0]

        print(
            "\nTotaled average: {avg} {unit}".format(
                avg=round(total_avg, 4), unit=self.unit
            )
        )


class JsonRailPrinter(RailStatsPrinter):
    def __init__(self, rails):
        pass

    def print(self, rail_stats):
        timestamp = int(time.time())
        json_fmt = (
            '{{"int":{{"time":{timestamp}}},'
            + '"normal":{{"rail":"{rail}"}},'
            + '"double":{{"Max {unit}":{max_watts},'
            + '"Min {unit}":{min_watts},'
            + '"Avg {unit}":{average}}}}}'
        )
        print(
            "\n".join(
                json_fmt.format(
                    timestamp=timestamp,
                    rail=rail.name,
                    average=stats[0],
                    min_watts=stats[1],
                    max_watts=stats[2],
                    unit=rail.unitStr,
                )
                for rail, stats in rail_stats.items()
            ),
        )


class RailHierarchyPrinter(RailStatsPrinter):
    styles = [style.BLUE, style.DARKCYAN, style.GREEN]

    def __init__(self, rails):
        self.rail_by_name = {}
        self.rails_by_parent = {}
        self.rail_stats = None

        for rail in rails:
            self.rail_by_name[rail.name] = rail
            if rail.parent not in self.rails_by_parent:
                self.rails_by_parent[rail.parent] = []
            self.rails_by_parent[rail.parent].append(rail)

    def validate_hierarchy_walk(self, rail, visited, known_decendents):
        if rail.name in visited:
            return
        visited.add(rail.name)
        if not rail.parent:
            return
        if rail.parent not in self.rail_by_name:
            raise ValueError(
                "[Rail='{}'] 'parent' = '{}' is unknown".format(rail.name, rail.parent)
            )
        if rail.parent in known_decendents:
            raise ValueError(
                "[Rail='{}'] is part of invalid hierarchy containing a cycle".format(
                    rail.name
                )
            )
        known_decendents.add(rail.name)
        self.validate_hierarchy_walk(
            self.rail_by_name[rail.parent], visited, known_decendents
        )

    def validate_hierarchy(self):
        # Make sure every parent rail exists and the graph contains no cycles.
        visited = set()
        for (_, rail) in self.rail_by_name.items():
            self.validate_hierarchy_walk(rail, visited, set())

    def print_walk(self, rail, level=0):
        fmt = "{subsys} - {avg} {unit} (avg), {min} {unit} (min), {max} {unit} (max)"
        prefix = level * "    " + "├── " if rail.parent else "└── "
        if level < len(RailHierarchyPrinter.styles):
            fmt = (
                prefix + RailHierarchyPrinter.styles[level] + "{rail}" + style.END + fmt
            )
        else:
            fmt = prefix + "{rail}" + fmt

        unit = rail.unitStr
        stats = self.rail_stats[rail]
        print(
            fmt.format(
                subsys=" ({})".format(rail.subsystem) if rail.subsystem else "",
                rail=rail.name,
                avg=round(stats[0], 4),
                min=round(stats[1], 4),
                max=round(stats[2], 4),
                unit=unit,
            )
        )

        if rail.name in self.rails_by_parent:
            for child_rail in self.rails_by_parent[rail.name]:
                self.print_walk(child_rail, level + 1)

    def print(self, rail_stats):
        self.rail_stats = rail_stats
        for rail in self.rails_by_parent[None]:
            self.print_walk(rail)


def find_all_usbdev(config, boards):
    global num_hops_to_dest_board

    def diag_dump_board_info():
        # Print a dump of board info, for diagnostic purposes
        eprint("-- Detected " + str(len(allusbdev)) + " Sentinel board(s) via USB --")

        for board in orderusbdev:
            if board:
                eprint("# Board " + str(orderusbdev.index(board)))
                try:
                    eprint(yaml.dump(sentinel_get_board_info(board)))
                except Exception as e:
                    eprint(e)
            else:
                eprint(board)
            eprint()

    # find USB Device for every Sentinel board
    allusbdev = list(
        usb.core.find(idVendor=SENTINEL_USB_VID, idProduct=SENTINEL_USB_PID, find_all=1)
    )
    orderusbdev = get_daisy_chain_order(allusbdev)
    if len(orderusbdev) == 0:
        raise ValueError("Sentinel Power Monitor USB device not found")

    if config.sync_uptimes:
        sentinel_sync_timestamp(orderusbdev)

    # Collect all board indices configred with at least one rail
    active_board_indices = [i for i in range(len(boards)) if len(boards[i]) > 0]

    # Treat the board index CLI argument as a "starting" index, from which YAML board
    # indices (zero if unspecified) are offset
    try:
        starting_index = int(config.board_index)
        if starting_index < 0:
            raise ValueError()
    except ValueError:
        diag_dump_board_info()
        raise ValueError("Invalid board index: " + str(config.board_index))

    # "Base" device is the one we tunnel through, if necessary
    # For now just populate with a default assuming we're not tunnneling
    base_index = starting_index
    base_usb_dev = None
    tunnel_hops = 0
    tunnel = False

    for i in active_board_indices:
        board_index = starting_index + i
        if board_index >= len(orderusbdev) or not orderusbdev[board_index]:
            # Target board is not USB-accessible
            # Try to reach it by tunneling from previous device
            if tunnel or len(active_board_indices) > 1:
                # It's going to take some rework to try tunneling w/multiple boards,
                # because we need to start using (device, hops) tuples everywhere.
                # Unsupported for now.
                eprint(
                    "Cannot reach board "
                    + str(board_index)
                    + " via USB, and tunneling not available."
                )
                raise ValueError("Only one board at a time may be used via tunneling.")

            # Change base to the previous board (with safeguards against out-of-bounds)
            base_index = max(min(len(orderusbdev), board_index) - 1, 0)
            base_usb_dev = orderusbdev[base_index]
            tunnel_hops = board_index - base_index
            tunnel = True

    # Release any boards we're not going to be using, and make sure rest are reachable
    for index in range(len(orderusbdev)):
        dev = orderusbdev[index]
        in_use = False

        if tunnel and dev == base_usb_dev:
            # We always use the "base" board for tunneling
            in_use = True

        if index - starting_index in active_board_indices:
            in_use = True

        # Verify that in-use device is available
        if in_use and not dev:
            eprint("Board " + str(index) + " not found or not responding via USB")
            diag_dump_board_info()
            raise Exception("Cannot communicate with target Sentinel board")

        if not in_use and dev:
            dev.finalize()

    if tunnel:
        # Check that the chain is actually that long
        while num_hops_to_dest_board < tunnel_hops:
            board_info_i = sentinel_get_board_info(base_usb_dev)
            # Verify that board has another attached
            if (board_info_i["board_up_dn"] & 32) == 0:  # TODO FIXME magic number
                diag_dump_board_info()
                raise Exception(
                    "Cannot find board #"
                    + str(base_index + tunnel_hops)
                    + " by daisy chain or USB"
                )

            num_hops_to_dest_board += 1

    # Snip prefix off orderusbdef so it starts with starting_index (this way YAML board
    # indices behave as normal), or base_index if tunneling.
    orderusbdev = orderusbdev[base_index:]

    return orderusbdev


def check_fw_compatiblity(usbdev, board_id):
    # get board info
    board_info = sentinel_get_board_info(usbdev)
    if board_info is None:
        raise ValueError("Failed to get Sentinel board info")

    # make sure this board matches the config
    if board_info["board_id"] != board_id:
        raise ValueError(
            "Board ID from device ({}) does not match config file id ({})".format(
                board_info["board_id"], board_id
            )
        )

    ver_maj = board_info["fw_ver_major"]
    ver_min = board_info["fw_ver_minor"]

    # no need to version numbers for pre-release firmware, which is always v0.0
    if is_prerelease_fw(board_info):
        return ver_maj, ver_min

    if FW_VER_MAJOR == 0 and FW_VER_MINOR == 0:
        static_msgs.append(
            "WARNING: Sentinel script version unknown! Can't guarantee compatibility."
        )
    elif ver_maj > FW_VER_MAJOR or (ver_maj == FW_VER_MAJOR and ver_min > FW_VER_MINOR):
        static_msgs.append(
            "WARNING: Board {} FW v{}.{} is newer than this script's version v{}.{}. "
            "Compatibility is NOT guaranteed. Use at your own risk (or get the latest "
            "version of this script from the FW release).".format(
                board_info["board_index"], ver_maj, ver_min, FW_VER_MAJOR, FW_VER_MINOR
            )
        )

    if not check_board_version_at_least(board_info, MIN_FW_VER_MAJOR, MIN_FW_VER_MINOR):
        raise ValueError(
            "Board FW v{}.{} is not compatible. Please update to at least FW v{}.{}".format(
                ver_maj,
                ver_min,
                MIN_FW_VER_MAJOR,
                MIN_FW_VER_MINOR,
            )
        )

    return ver_maj, ver_min


def read_data_thread(
    board, usbdev, rails, consumers, timebase, error_abort=False, timesync_trigger=None
):
    global is_exiting
    global is_recording

    # Set the zero point timestamp for all rails on the board.
    board_info = sentinel_get_board_info(usbdev)
    board_zero = board_info["uptime"]
    sys_zero = board_zero  # by default, use the board's timebase
    if timebase == "host":
        sys_zero = time.time() * 1000000

    for rail in rails:
        rail.set_timebase(board_zero, sys_zero)

    sentinel_streaming_on_off(usbdev, True)
    error_count = {err: 0 for err in SENTINEL_MSG_ERROR_TYPES}
    last_alert = {err: 0 for err in SENTINEL_MSG_ERROR_TYPES}
    is_recording = timesync_trigger is None  # record right away if no trigger
    while True:
        if is_exiting:
            return
        try:
            buf = usbdev.read(USB_FROM_DEV_DATA_EP, 64)
        except Exception as e:
            try:
                # The data endpoint failed (very likely timed out).
                # Is there data on the config endpoint?
                # The 0.8.0 firmware uses this to warn of accumulating error conditions
                # (only schedule misses, though). HOWEVER, this has the extremely
                # unwanted behavior of waiting on a 1sec timeout and hence 1s hang every
                # time such a warning message is sent! So, this has been removed in more
                # recent firmwares. But, the handler remains for 0.8.0 compatibility.
                msg = usbdev.read(USB_FROM_DEV_CFG_EP, 0xFFFF, timeout=100)
                eprint(
                    "On-device error count warning ({}): {}".format(
                        str(e), str(str(unpack_rsp(msg)))
                    )
                )
                if not error_abort:
                    continue
                else:
                    break
            except Exception as e:
                eprint("\nBoard {} {}".format(str(board), str(e)))
                break

        msg = unpack_msg(buf, rails)
        if msg["msgid"] == SENTINEL_MSG_DATA_STREAM:
            if msg["errors"]:
                alerts = []
                board_err_msg = ""
                for error in error_count.keys():
                    if error in msg["errors"]:
                        error_count[error] += ERROR_XMIT_THRESHOLDS[error]
                    if error_count[error] > 0:
                        alert = "\t{} (total: {})".format(error, error_count[error])
                        board_err_msg += alert
                    if error_count[error] - last_alert[error] > ERROR_COUNT_THRESHOLD:
                        alerts.append(alert)
                        last_alert[error] = error_count[error]
                if alerts:
                    # Send a message to stderr
                    eprint(
                        "On-device error count warning for board {}.\n"
                        "Thresholds passed for error type(s):\n{}".format(
                            str(board), "\n".join(alerts)
                        )
                    )
                # Post an updated banner in the text UI (if visible)
                board_errors[board] = board_err_msg

                if error_abort:
                    break

            for (rail_id, timestamp, sample) in msg["samples"]:
                rail = get_rail(rails, rail_id)
                rail.set_sample(timestamp, sample)
                if (
                    not is_recording
                    and rail_id == timesync_trigger["rail_id"]
                    and rail.value > timesync_trigger["threshold"]
                ):
                    eprint("Recording starting...")
                    # Reset timebase for all rails to indicate collection is starting now.
                    for rail in rails:
                        rail.set_timebase(rail.last_timestamp, 0)
                    is_recording = True

                if is_recording:
                    for consumer in consumers:
                        consumer.update_rail(rail)


def all_read_data_threads_running(all_read_threads):
    for t in all_read_threads:
        if not t.is_alive():
            return False
    return True


def end_all_read_threads(all_read_threads):
    global is_exiting

    is_exiting = True
    for t in all_read_threads:
        t.join()

    return True


class Sentinel:
    def __init__(
        self,
        config,
    ):

        self.all_rails = []
        self.all_root_rails = []
        self.consumers = []
        self.ver_maj = None
        self.ver_min = None
        self.update_count = (
            0  # Number of times the main thread while loop has updated the display
        )

        if config:
            self.config = config
        else:
            self.config = SentinelConfig()

        self.all_read_threads = []

        # check connection, before talking to board
        if self.config.connect_pts:
            addr, port = self.config.connect_pts.split(":")
            sock = open_socket(addr, int(port))

        if self.config.test:
            (
                self.board_name,
                self.board_id,
                self.boards,
                timesync_trigger_config,
            ) = sentinel_INA_test_config(self.config.yaml_file, self.config.sample_rate)
        else:
            (
                self.board_name,
                self.board_id,
                self.boards,
                timesync_trigger_config,
            ) = load_config(self.config.yaml_file, self.config)

        self.orderusbdev = find_all_usbdev(self.config, self.boards)

        if (self.config.configure_only) and (self.config.capture_only):
            # Cannot enable both!
            raise Exception(
                "Cannot turn on both config only (-f) and capture only (-z)!"
            )

        for rails in self.boards:
            if len(rails) > 0:
                try:
                    devnum = self.boards.index(rails)
                    usbdev = self.orderusbdev[devnum]
                except IndexError:
                    devnum = 0
                    usbdev = self.orderusbdev[devnum]

                if not self.config.capture_only:
                    self.ver_maj, self.ver_min = check_fw_compatiblity(
                        usbdev, self.board_id
                    )
                    sentinel_cfg_reset(usbdev)
                    sentinel_INA_reset(usbdev)
                    sentinel_streaming_on_off(usbdev, False)
                    # Read any last unread data from usb
                    try:
                        usbdev.read(USB_FROM_DEV_DATA_EP, 0xFFFF, timeout=100)
                    except usb.core.USBError:
                        pass

                for rail_id, rail in enumerate(rails):
                    rail.usbdevnum = devnum
                    rail.set_id(rail_id)
                    sentinel_cfg_rail(usbdev, rail, (not self.config.capture_only))
                    self.all_rails.append(rail)
                    self.all_root_rails.append(rail)

                    for sub_rail in rail.sub_rails:
                        sub_rail.usbdevnum = devnum
                        sentinel_cfg_rail(
                            usbdev, sub_rail, (not self.config.capture_only)
                        )

                        self.all_rails.append(sub_rail)

        self.dump_sync_file = None
        if self.config.csv_dump_file:
            if self.config.csv_dump_file == "-":
                self.dump_sync_file = sys.stdout
            else:
                self.dump_sync_file = open(self.config.csv_dump_file, "w")

            self.consumers.append(
                NormalizedTimeCsvDumper(self.all_rails, self.dump_sync_file)
            )

        if self.config.scope:
            self.scope = LiveSampleScope(
                self.all_rails,
                max_history=int(self.config.plot_samples),
                ymax=self.config.ymax,
            )
            self.consumers.append(self.scope)

        if self.config.calc_stats:
            if not self.config.length_seconds:
                raise ValueError(
                    "--length-seconds must be specified to calculate stats"
                )
            printer = None
            if self.config.printer == "hierarchy":
                printer = RailHierarchyPrinter(self.all_rails)
                printer.validate_hierarchy()
            elif self.config.printer == "json":
                printer = JsonRailPrinter(self.all_rails)
            else:
                printer = StandardRailPrinter(self.all_rails)
            self.consumers.append(
                SummarizingDataAnalyzer(
                    self.all_rails, usbdev, float(self.config.length_seconds), printer
                )
            )
        if self.config.connect_pts:
            self.consumers.append(NormalizedTimeCsvTcpIp(self.all_rails, sock))
        if self.config.test:
            self.consumers.append(
                TestDataAnalyzer(
                    self.all_rails,
                    self.config.yaml_file,
                    usbdev,
                    self.config.sample_rate,
                )
            )

        self.timesync_trigger = None
        if timesync_trigger_config:
            self.timesync_trigger = {"threshold": timesync_trigger_config["threshold"]}
            mode = ina_mode[timesync_trigger_config["mode"]]
            for rail in self.boards[timesync_trigger_config["board"]]:
                if rail.name == timesync_trigger_config["rail"]:
                    if rail.mode == mode:
                        self.timesync_trigger["rail_id"] = rail.rail_id
                        break
                    for sub_rail in rail.sub_rails:
                        if sub_rail.mode == mode:
                            self.timesync_trigger["rail_id"] = sub_rail.rail_id
                            break
                    break
            if "rail_id" not in self.timesync_trigger:
                raise ValueError(
                    "'timesync_trigger' defined in config refers to a rail not defined in 'rails'"
                )

        if self.config.flag_file_path:
            open(self.config.flag_file_path, "w").close()

    def get_sentinel_ts(self):
        return get_sentinel_timestamp(self.orderusbdev)

    def start(self):

        self.all_read_threads = []
        for rails in self.boards:
            if len(rails) > 0:
                board = self.boards.index(rails)
                try:
                    usbdev = self.orderusbdev[board]
                except IndexError:
                    usbdev = self.orderusbdev[0]
                t = threading.Thread(
                    target=read_data_thread,
                    args=(
                        board,
                        usbdev,
                        rails,
                        self.consumers,
                        self.config.timebase if self.config.timebase else "sentinel",
                        self.config.abort_on_error,
                        self.timesync_trigger,
                    ),
                )
                t.start()
                self.all_read_threads.append(t)

        global is_recording
        start_time = time.time()
        last_update_time = start_time - 0.15  # W/fudge factor to reduce startup latency
        while all_read_data_threads_running(self.all_read_threads):
            now = time.time()
            self.abort_if_flag()
            if (
                (self.dump_sync_file != sys.stdout)
                and not self.config.quiet_mode
                and is_recording
            ):
                if now - last_update_time > 0.25:
                    update_display(
                        self.board_name,
                        self.board_id,
                        self.ver_maj,
                        self.ver_min,
                        self.all_root_rails,
                        self.config.verbose,
                    )
                    last_update_time = now

            if self.config.scope and self.scope.ready_to_redraw:
                self.scope.redraw()

            if (float(self.config.length_seconds) != 0) and (
                now - start_time > float(self.config.length_seconds)
            ):
                break

            time.sleep(0.1)

        self.abort_run()

        for consumer in self.consumers:
            consumer.complete()

        if self.dump_sync_file and self.dump_sync_file != sys.stdout:
            self.dump_sync_file.close()

        self.reset()

    def abort_run(self):
        # Stop read threads -- normally this is called automatically as start() returns,
        # but it can be called for cleanup or interruption, including from another thread.
        end_all_read_threads(self.all_read_threads)

    def reset(self):
        # Reset Sentinel hardware
        for rails in self.boards:
            if len(rails) > 0:
                try:
                    usbdev = self.orderusbdev[self.boards.index(rails)]
                except IndexError:
                    usbdev = self.orderusbdev[0]
                try:
                    sentinel_INA_reset(usbdev)
                    sentinel_cfg_reset(usbdev)
                except usb.core.USBError:
                    # If usb down due to already reset fall through.
                    pass

    def abort_if_flag(self):
        # if the flag_file is 0, then terminate
        # check every 10 updates
        if self.config.flag_file_path:
            if self.update_count % 10 == 0:
                try:
                    with open(self.config.flag_file_path, "r") as flag_file:
                        content = flag_file.read()
                        if "0" in content:
                            print(
                                f"Stopping Sentinel run after {self.update_count} updates"
                            )
                            self.abort_run()
                except IOError:
                    open(self.config.flag_file_path, "w")

        self.update_count += 1


def parse_args():
    parser = argparse.ArgumentParser()
    h = "Path to yaml config file"
    parser.add_argument("-c", "--config", help=h, required=True)
    h = "Index of board to connect to (if multiple)"
    parser.add_argument("-b", "--boardindex", help=h, default=0)
    h = "Dump data to CSV file with 'synchronized' timestamp using interpolation (or '-' for stdout)"
    parser.add_argument("-d", "--dump-sync", help=h)
    h = "Show live scope-style plot (requires matplotlib)"
    parser.add_argument("-s", "--scope", help=h, action="store_true", default=False)
    h = "Y-axis limit for live plots"
    parser.add_argument("-y", "--ymax", help=h)
    h = "Number of samples for live plots (max size of sliding window)"
    parser.add_argument("-n", "--plot-samples", help=h, default=200)
    h = "Show verbose output including sample count and timing"
    parser.add_argument("-v", "--verbose", help=h, action="store_true", default=False)
    h = "Specify sample rate in Hz for all rails (overriding YAML)"
    parser.add_argument("-r", "--sample-rate", help=h)
    h = "IP Address and Port- <addr:port>"
    parser.add_argument("-p", "--connect_pts", help=h)
    h = "Launch automated test run (requires a sampling mode under -c)"
    parser.add_argument("-t", "--test", help=h, action="store_true", default=False)
    h = "Select sampling mode (overrides YAML 'mode')"
    parser.add_argument("-m", "--modesample", help=h)
    h = "Synchronize all boards to the same timestamp"
    parser.add_argument(
        "-u", "--uptimesync", help=h, action="store_true", default=False
    )
    h = "Disable the live power measurement display (stdout text UI)"
    parser.add_argument(
        "-q", "--quiet-mode", help=h, action="store_true", default=False
    )
    h = "Configure sentinel only, without starting data capture"
    parser.add_argument("-f", "--cfg_only", help=h, action="store_true", default=False)
    h = "Start data capture immediately, assuming sentinel pre-configured"
    parser.add_argument("-z", "--cap_only", help=h, action="store_true", default=False)
    h = "Abort on error"
    parser.add_argument(
        "-a", "--abort_on_error", help=h, action="store_true", default=False
    )
    h = "Specify the number of seconds to capture data before exiting"
    parser.add_argument("-l", "--length-seconds", help=h, default=0)
    h = "Calculate stats for each rail (mean, min, max); requires --length-seconds to be specified"
    parser.add_argument(
        "-e", "--calc-stats", help=h, action="store_true", default=False
    )
    h = "Printer to use for the rail stats, valid options: standard, hierarchy; ignored when --calc-stats is not specified"
    parser.add_argument("-i", "--printer", help=h, default="standard")
    h = "Include only rails with YAML matching regular expression"
    parser.add_argument("-F", "--filter", help=h)
    h = "Zero point to use when emitting sample timestamps, valid options: 'sentinel', 'host, 'trigger';"
    "'host' is the host machine's clock, which is usec since the unix epoch."
    "'sentinel' is the usec since the sentinel board booted."
    "'trigger' will delay recording until a signal is received from the device, then use that as the zero point."
    parser.add_argument("-T", "--timebase", help=h, default="sentinel")

    return parser.parse_args()


def create_parsed_config():
    args = parse_args()
    config = SentinelConfig(vars(args))

    return config


def main():

    global static_msgs
    global FW_VER_MAJOR
    global FW_VER_MINOR
    global streaming_on
    global is_exiting

    if FW_VER_MAJOR == 0 and FW_VER_MINOR == 0:
        try:
            with open("version", "r") as f:
                expected_ver = f.read().strip().split(".")
                exp_ver_maj = int(expected_ver[0], 0)
                exp_ver_min = int(expected_ver[1], 0)
                FW_VER_MAJOR = exp_ver_maj
                FW_VER_MINOR = exp_ver_min
        except Exception:
            pass

    sentinel = None
    try:
        config = create_parsed_config()
        sentinel = Sentinel(config)
        if not config.configure_only:
            try:
                sentinel.start()

            except KeyboardInterrupt:
                sentinel.abort_run()
                sentinel.reset()
                return

    except Exception as e:
        eprint("[ERROR] {}".format(str(e)))
        raise e


if __name__ == "__main__":
    main()
