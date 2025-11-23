from machine import Pin, I2C
import ustruct
import time
from collections import namedtuple

StatusBits = namedtuple("StatusBits", "zyxda zyxda zyxor zor yor xor drdy reserved")
CfgAFields = namedtuple("CfgAFields", "comp_temp_en reboot soft_rst lp odr md")
CfgBFields = namedtuple("CfgBFields", "one_shot int_off set_freq off_canc lpf")
CfgCFields = namedtuple("CfgCFields", "int_on_pin i2c_dis bdu ble fourwspi self_test drdy_on_pin")
IntCtrlBits = namedtuple("IntCtrlBits", "xien yien zien iea iel ien")
IntSourceBits = namedtuple("IntSourceBits", "p_th_s_x p_th_s_y p_th_s_z n_th_s_x n_th_s_y n_th_s_z mroi int")

class LIS2MDL():
    # default I2C address
    __LIS2MDL_ADDR = 0x1E

    # Register Map
    __OFFSET_X_REG_L = 0x45
    __OFFSET_X_REG_H = 0x46
    __OFFSET_Y_REG_L = 0x47
    __OFFSET_Y_REG_H = 0x48
    __OFFSET_Z_REG_L = 0x49
    __OFFSET_Z_REG_H = 0x4A
    __WHO_AM_I = 0x4F
    __CFG_REG_A = 0x60
    __CFG_REG_B = 0x61
    __CFG_REG_C = 0x62

    __INT_CRTL_REG = 0x63
    __INT_SOURCE_REG = 0x64
    __INT_THS_L_REG = 0x65
    __INT_THS_H_REG = 0x66

    __STATUS_REG = 0x67

    __OUTX_L_REG = 0x68
    __OUTX_H_REG = 0x69
    __OUTY_L_REG = 0x6A
    __OUTY_H_REG = 0x6B
    __OUTZ_L_REG = 0x6C
    __OUTZ_H_REG = 0x6D
    __TEMP_OUT_L_REG = 0x6E
    __TEMP_OUT_H_REG = 0x6F

    # --- init ---
    def __init__(self, i2c_addr = __LIS2MDL_ADDR, i2c_id = 0, scl_pin = 17, sda_pin = 16, freq=400000):
        # Set i2c pins and initialize
        self.i2c = I2C(i2c_id, scl=Pin(scl_pin), sda=Pin(sda_pin), freq=freq)
        self.i2c_addr = i2c_addr

    # ======================
    # Low-level application for register access
    # ======================
    def read_reg(self, reg, nbytes=1):
        """讀取 register 回傳 bytes"""
        return self.i2c.readfrom_mem(self.i2c_addr, reg, nbytes)

    def write_reg(self, reg, data_bytes):
        """寫入 register (data_bytes: bytes or bytearray)"""
        self.i2c.writeto_mem(self.i2c_addr, reg, data_bytes)

    # convenience wrappers for common registers
    def read_offset_x_reg(self):
        return self.read_reg(self.__OFFSET_X_REG_L, 2)

    def read_offset_y_reg(self):
        return self.read_reg(self.__OFFSET_Y_REG_L, 2)

    def read_offset_z_reg(self):
        return self.read_reg(self.__OFFSET_Z_REG_L, 2)

    def read_who_am_i(self):
        return self.read_reg(self.__WHO_AM_I, 1)

    def read_cfg_a_reg(self):
        return self.read_reg(self.__CFG_REG_A, 1)

    def read_cfg_b_reg(self):
        return self.read_reg(self.__CFG_REG_B, 1)

    def read_cfg_c_reg(self):
        return self.read_reg(self.__CFG_REG_C, 1)

    def read_int_ctrl_reg(self):
        return self.read_reg(self.__INT_CRTL_REG, 1)

    def read_int_source_reg(self):
        return self.read_reg(self.__INT_SOURCE_REG, 1)

    def read_int_ths_reg(self):
        return self.read_reg(self.__INT_THS_L_REG, 2)

    def read_status_reg(self):
        return self.read_reg(self.__STATUS_REG, 1)

    def read_outx_reg(self):
        return self.read_reg(self.__OUTX_L_REG, 2)

    def read_outy_reg(self):
        return self.read_reg(self.__OUTY_L_REG, 2)

    def read_outz_reg(self):
        return self.read_reg(self.__OUTZ_L_REG, 2)

    def read_temp_out_reg(self):
        return self.read_reg(self.__TEMP_OUT_L_REG, 2)

    # ======================
    # High-level application for raw data access
    # ======================
    # offsets (signed 16)
    def get_offset_x(self):
        b = self.read_offset_x_reg()
        return ustruct.unpack("<h", b)[0]

    def get_offset_y(self):
        b = self.read_offset_y_reg()
        return ustruct.unpack("<h", b)[0]

    def get_offset_z(self):
        b = self.read_offset_z_reg()
        return ustruct.unpack("<h", b)[0]

    # who am i (byte)
    def get_who_am_i(self):
        b = self.read_who_am_i()
        return ustruct.unpack("B", b)[0]

    # CFG A
    def get_cfg_a(self):
        b = self.read_cfg_a_reg()
        val = ustruct.unpack("B", b)[0]
        # COMP_TEMP_EN (bit7), REBOOT (6), SOFT_RST (5), LP (4), ODR1:0 (bits3-2), MD1:0 (bits1-0)
        comp_temp_en = (val >> 7) & 0x1
        reboot = (val >> 6) & 0x1
        soft_rst = (val >> 5) & 0x1
        lp = (val >> 4) & 0x1
        odr = (val >> 2) & 0x3
        md = val & 0x3
        return CfgAFields(comp_temp_en, reboot, soft_rst, lp, odr, md)

    # CFG B
    def get_cfg_b(self):
        b = self.read_cfg_b_reg()
        val = ustruct.unpack("B", b)[0]
        # bits: OFF_CANC_ONE_SHOT (4), INT_OFF (3), SET_FREQ (2), OFF_CANC (1), LPF (0)
        one_shot = (val >> 4) & 0x1
        int_off = (val >> 3) & 0x1
        set_freq = (val >> 2) & 0x1
        off_canc = (val >> 1) & 0x1
        lpf = val & 0x1
        return CfgBFields(one_shot, int_off, set_freq, off_canc, lpf)

    # CFG C
    def get_cfg_c(self):
        b = self.read_cfg_c_reg()
        val = ustruct.unpack("B", b)[0]
        # bits: INT_on_PIN (6), I2C_DIS (5), BDU (4), BLE (3), 4WSPI (2), SELF_TEST (1), DRDY_on_PIN (0)
        int_on_pin = (val >> 6) & 0x1
        i2c_dis = (val >> 5) & 0x1
        bdu = (val >> 4) & 0x1
        ble = (val >> 3) & 0x1
        fourwspi = (val >> 2) & 0x1
        self_test = (val >> 1) & 0x1
        drdy_on_pin = val & 0x1
        return CfgCFields(int_on_pin, i2c_dis, bdu, ble, fourwspi, self_test, drdy_on_pin)

    # INT_CTRL (bits mapping assumed: XIEN(7), YIEN(6), ZIEN(5), IEA(2), IEL(1), IEN(0))
    def get_int_ctrl(self):
        b = self.read_int_ctrl_reg()
        val = ustruct.unpack("B", b)[0]
        return IntCtrlBits((val >> 7) & 0x1, (val >> 6) & 0x1, (val >> 5) & 0x1,
                           (val >> 2) & 0x1, (val >> 1) & 0x1, val & 0x1)

    # INT SOURCE (bits order used below: P_TH_S_X (7) ... INT (0))
    def get_int_source(self):
        b = self.read_int_source_reg()
        val = ustruct.unpack("B", b)[0]
        return IntSourceBits((val >> 7) & 0x1,
                             (val >> 6) & 0x1,
                             (val >> 5) & 0x1,
                             (val >> 4) & 0x1,
                             (val >> 3) & 0x1,
                             (val >> 2) & 0x1,
                             (val >> 1) & 0x1,
                             val & 0x1)

    def get_int_ths(self):
        b = self.read_int_ths_reg()
        return ustruct.unpack("<h", b)[0]

    def get_status(self):
        b = self.read_status_reg()
        val = ustruct.unpack("B", b)[0]
        # Zyxor(7) Zor(6) Yor(5) Xor(4) Zyxda(3) zda(2) yda(1) xda(0) <-- adjust to datasheet if needed
        # you originally had different mapping; keep this mapping but user may adjust per datasheet
        return StatusBits((val >> 7) & 0x1, (val >> 6) & 0x1, (val >> 5) & 0x1,
                          (val >> 4) & 0x1, (val >> 3) & 0x1, (val >> 2) & 0x1,
                          (val >> 1) & 0x1, val & 0x1)

    # magnetic outputs and temp
    def get_mag_x(self):
        b = self.read_outx_reg()
        return ustruct.unpack("<h", b)[0]

    def get_mag_y(self):
        b = self.read_outy_reg()
        return ustruct.unpack("<h", b)[0]

    def get_mag_z(self):
        b = self.read_outz_reg()
        return ustruct.unpack("<h", b)[0]

    def get_temp(self):
        b = self.read_temp_out_reg()
        return ustruct.unpack("<h", b)[0]

    def get_mag_xyz(self):
        return self.get_mag_x(), self.get_mag_y(), self.get_mag_z()

    # ======================
    # Low-level application for register manipulation
    # ======================
    def write_offset_x_reg(self, data_bytes):
        self.write_reg(self.__OFFSET_X_REG_L, data_bytes)

    def write_offset_y_reg(self, data_bytes):
        self.write_reg(self.__OFFSET_Y_REG_L, data_bytes)

    def write_offset_z_reg(self, data_bytes):
        self.write_reg(self.__OFFSET_Z_REG_L, data_bytes)

    def write_cfg_reg_a(self, data_byte):
        self.write_reg(self.__CFG_REG_A, data_byte)

    def write_cfg_reg_b(self, data_byte):
        self.write_reg(self.__CFG_REG_B, data_byte)

    def write_cfg_reg_c(self, data_byte):
        self.write_reg(self.__CFG_REG_C, data_byte)

    def write_int_ctrl_reg(self, data_byte):
        self.write_reg(self.__INT_CRTL_REG, data_byte)

    def write_int_ths_reg(self, data_bytes):
        self.write_reg(self.__INT_THS_L_REG, data_bytes)

    # ======================
    # Register bits manipulation
    # set_bits: bits to set to 1
    # clear_mask: mask of bits that should be cleared to 0
    # reg: target register
    # ======================
    def set_reg_bits(self, set_bits, clear_mask, reg):
        cur = self.read_reg(reg, 1)
        val = ustruct.unpack("B", cur)[0]
        val = (val & (~clear_mask & 0xFF)) | (set_bits & 0xFF)
        newb = ustruct.pack("B", val)
        self.write_reg(reg, newb)

    # ======================
    # High-level application for setting
    # ======================
    def set_offset_x(self, offset):
        data_bytes = ustruct.pack("<h", offset)
        self.write_offset_x_reg(data_bytes)

    def set_offset_y(self, offset):
        data_bytes = ustruct.pack("<h", offset)
        self.write_offset_y_reg(data_bytes)

    def set_offset_z(self, offset):
        data_bytes = ustruct.pack("<h", offset)
        self.write_offset_z_reg(data_bytes)

    def set_cfg_a(self,
                  comp_temp_en = None,
                  reboot = None,
                  soft_rst = None,
                  lp = None,
                  odr = None,
                  md = None):
        set_bits = 0x00
        clear_mask = 0x00
        if comp_temp_en is not None:
            set_bits |= (comp_temp_en & 0x1) << 7
            clear_mask |= 0x1 << 7
        if reboot is not None:
            set_bits |= (reboot & 0x1) << 6
            clear_mask |= 0x1 << 6
        if soft_rst is not None:
            set_bits |= (soft_rst & 0x1) << 5
            clear_mask |= 0x1 << 5
        if lp is not None:
            set_bits |= (lp & 0x1) << 4
            clear_mask |= 0x1 << 4
        if odr is not None:
            set_bits |= (odr & 0x3) << 2
            clear_mask |= 0x3 << 2
        if md is not None:
            set_bits |= (md & 0x3)
            clear_mask |= 0x3
        self.set_reg_bits(set_bits, clear_mask, self.__CFG_REG_A)

    def set_cfg_b(self,
                  off_canc_one_shot = None,
                  int_on_data_off = None,
                  set_freq = None,
                  off_canc = None,
                  lpf = None):
        set_bits = 0x00
        clear_mask = 0x00
        if off_canc_one_shot is not None:
            set_bits |= (off_canc_one_shot & 0x1) << 4
            clear_mask |= 0x1 << 4
        if int_on_data_off is not None:
            set_bits |= (int_on_data_off & 0x1) << 3
            clear_mask |= 0x1 << 3
        if set_freq is not None:
            set_bits |= (set_freq & 0x1) << 2
            clear_mask |= 0x1 << 2
        if off_canc is not None:
            set_bits |= (off_canc & 0x1) << 1
            clear_mask |= 0x1 << 1
        if lpf is not None:
            set_bits |= (lpf & 0x1)
            clear_mask |= 0x1
        self.set_reg_bits(set_bits, clear_mask, self.__CFG_REG_B)

    def set_cfg_c(self,
                  int_on_pin = None,
                  i2c_dis = None,
                  bdu = None,
                  ble = None,
                  fourwspi = None,
                  self_test = None,
                  drdy_on_pin = None):
        set_bits = 0x00
        clear_mask = 0x00
        if int_on_pin is not None:
            set_bits |= (int_on_pin & 0x1) << 6
            clear_mask |= 0x1 << 6
        if i2c_dis is not None:
            set_bits |= (i2c_dis & 0x1) << 5
            clear_mask |= 0x1 << 5
        if bdu is not None:
            set_bits |= (bdu & 0x1) << 4
            clear_mask |= 0x1 << 4
        if ble is not None:
            set_bits |= (ble & 0x1) << 3
            clear_mask |= 0x1 << 3
        if fourwspi is not None:
            set_bits |= (fourwspi & 0x1) << 2
            clear_mask |= 0x1 << 2
        if self_test is not None:
            set_bits |= (self_test & 0x1) << 1
            clear_mask |= 0x1 << 1
        if drdy_on_pin is not None:
            set_bits |= (drdy_on_pin & 0x1)
            clear_mask |= 0x1
        self.set_reg_bits(set_bits, clear_mask, self.__CFG_REG_C)

    def set_int_ctrl(self,
                     xien = None,
                     yien = None,
                     zien = None,
                     iea = None,
                     iel = None,
                     ien = None):
        set_bits = 0x00
        clear_mask = 0x00
        if xien is not None:
            set_bits |= (xien & 0x1) << 7
            clear_mask |= 0x1 << 7
        if yien is not None:
            set_bits |= (yien & 0x1) << 6
            clear_mask |= 0x1 << 6
        if zien is not None:
            set_bits |= (zien & 0x1) << 5
            clear_mask |= 0x1 << 5
        if iea is not None:
            set_bits |= (iea & 0x1) << 2
            clear_mask |= 0x1 << 2
        if iel is not None:
            set_bits |= (iel & 0x1) << 1
            clear_mask |= 0x1 << 1
        if ien is not None:
            set_bits |= (ien & 0x1)
            clear_mask |= 0x1
        self.set_reg_bits(set_bits, clear_mask, self.__INT_CRTL_REG)