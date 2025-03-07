/**
 * @file EVE_GpuDefs.h
 * @brief Defines EVE hardware values
 *
 * This header is separated and included last
 * in case of conflicts with other libraries.
 *
 * Expects BT_81XA_ENABLE, BT_81X_ENABLE, BT_88X_ENABLE, FT_81X_ENABLE, or FT_80X_ENABLE
 * to be defined. If not, multi target compilation is assumed.
 *
 * @author Bridgetek
 *
 * @date 2018
 *
 * MIT License
 *
 * Copyright (c) [2019] [Bridgetek Pte Ltd (BRTChip)]
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef EVE_GPU_DEFS__H
#define EVE_GPU_DEFS__H

/***********
** MARCOS **
***********/

#define EVE_DL_SIZE        (8 * 1024UL) /**< 8kB Display List buffer size */
#define EVE_DL_COUNT       (2 * 1024UL)
#define EVE_CMD_FIFO_SIZE  ((4) * 1024UL) /**< 4kB coprocessor FIFO size */
#define EVE_CMD_FIFO_COUNT (1024UL)
#define EVE_CMD_FIFO_MASK  (EVE_CMD_FIFO_SIZE - 1)
#define EVE_CMD_FIFO_ALIGNMENT_MASK (EVE_CMD_FIFO_SIZE - ((4) - 1))

#define EVE_CMD_FAULT(rp) (rp & 0x3)

/** @name Addresses */
///@{

#define RAM_G           0UL
#define ROM_CHIPID      786432UL /**< 0xC0000 */
#define ROM_FONT        0x1E0000
#define ROM_FONTROOT    0x2FFFFC
#define RAM_DL          0x300000
#define RAM_REG         0x302000
#define RAM_CMD         0x308000
#define RAM_ERR_REPORT  0x309800

#define RAM_G_SIZE (1024 * 1024L)

/**@name Graphics Engine Registers */
///@{
#define REG_HCYCLE              (RAM_REG + 0x2C) /**< 0x30202C */
#define REG_HOFFSET             (RAM_REG + 0x30) /**< 0x302030 */
#define REG_HSIZE               (RAM_REG + 0x34) /**< 0x302034 */
#define REG_HSYNC0              (RAM_REG + 0x38) /**< 0x302038 */
#define REG_HSYNC1              (RAM_REG + 0x3C) /**< 0x30203C */
#define REG_VCYCLE              (RAM_REG + 0x40) /**< 0x302040 */
#define REG_VOFFSET             (RAM_REG + 0x44) /**< 0x302044 */
#define REG_VSIZE               (RAM_REG + 0x48) /**< 0x302048 */
#define REG_VSYNC0              (RAM_REG + 0x4C) /**< 0x30204C */
#define REG_VSYNC1              (RAM_REG + 0x50) /**< 0x302050 */
#define REG_DLSWAP              (RAM_REG + 0x54) /**< 0x302054 */
#define REG_ROTATE              (RAM_REG + 0x58) /**< 0x302058 */
#define REG_OUTBITS             (RAM_REG + 0x5C) /**< 0x30205C */
#define REG_DITHER              (RAM_REG + 0x60) /**< 0x302060 */
#define REG_SWIZZLE             (RAM_REG + 0x64) /**< 0x302064 */
#define REG_CSPREAD             (RAM_REG + 0x68) /**< 0x302068 */
#define REG_PCLK_POL            (RAM_REG + 0x6C) /**< 0x30206C */
#define REG_PCLK                (RAM_REG + 0x70) /**< 0x302070 */
#define REG_TAG_X               (RAM_REG + 0x74) /**< 0x302074 */
#define REG_TAG_Y               (RAM_REG + 0x78) /**< 0x302078 */
#define REG_TAG                 (RAM_REG + 0x7C) /**< 0x30207C */
#define REG_HSF_HSIZE           (RAM_REG + 0x624) /**< 0x302624 */
///@}

/**@name Audio Engine Registers */
///@{
#define REG_VOL_PB              (RAM_REG + 0x80) /**< 0x302080 */
#define REG_VOL_SOUND           (RAM_REG + 0x84) /**< 0x302084 */
#define REG_SOUND               (RAM_REG + 0x88) /**< 0x302088 */
#define REG_PLAY                (RAM_REG + 0x8C) /**< 0x30208C */
#define REG_PLAYBACK_START      (RAM_REG + 0xB4) /**< 0x3020B4 */
#define REG_PLAYBACK_LENGTH     (RAM_REG + 0xB8) /**< 0x3020B8 */
#define REG_PLAYBACK_READPTR    (RAM_REG + 0xBC) /**< 0x3020BC */
#define REG_PLAYBACK_FREQ       (RAM_REG + 0xC0) /**< 0x3020C0 */
#define REG_PLAYBACK_FORMAT     (RAM_REG + 0xC4) /**< 0x3020C4 */
#define REG_PLAYBACK_LOOP       (RAM_REG + 0xC8) /**< 0x3020C8 */
#define REG_PLAYBACK_PLAY       (RAM_REG + 0xCC) /**< 0x3020CC */
#define REG_PLAYBACK_PAUSE      (RAM_REG + 0x5EC) /**< 0x3025EC */
///@}

/**@name Flash Registers */
///@{
#define REG_FLASH_STATUS        (RAM_REG + 0x5F0) /**< 0x3025F0 */
#define REG_FLASH_SIZE          (RAM_REG + 0x7024) /**< 0x309024 */
///@}

/**@name Touch Screen Engine Regieters */
///@{
#define REG_TOUCH_TRANSFORM_A   (RAM_REG + 0x150) /**< 0x302150 */
#define REG_TOUCH_TRANSFORM_B   (RAM_REG + 0x154) /**< 0x302154 */
#define REG_TOUCH_TRANSFORM_C   (RAM_REG + 0x158) /**< 0x302158 */
#define REG_TOUCH_TRANSFORM_D   (RAM_REG + 0x15C) /**< 0x30215C */
#define REG_TOUCH_TRANSFORM_E   (RAM_REG + 0x160) /**< 0x302160 */
#define REG_TOUCH_TRANSFORM_F   (RAM_REG + 0x164) /**< 0x302164 */
#define REG_TOUCH_CONFIG        (RAM_REG + 0x168) /**< 0x302168 */

#define REG_TOUCH_MODE          (RAM_REG + 0x104) /**< 0x302104 */
#define REG_CTOUCH_EXTENDED     (RAM_REG + 0x108) /**< 0x302108 */
#define REG_TOUCH_ADC_MODE      (RAM_REG + 0x108) /**< 0x302108 */
#define REG_TOUCH_CHARGE        (RAM_REG + 0x10C) /**< 0x30210C */
#define REG_TOUCH_SETTLE        (RAM_REG + 0x110) /**< 0x302110 */
#define REG_TOUCH_OVERSAMPLE    (RAM_REG + 0x114) /**< 0x302114 */
#define REG_TOUCH_RZTHRESH      (RAM_REG + 0x118) /**< 0x302118 */
#define REG_CTOUCH_TOUCH1_XY    (RAM_REG + 0x11C) /**< 0x30211C */
#define REG_TOUCH_RAW_XY        (RAM_REG + 0x11C) /**< 0x30211C */

#define REG_TOUCH_DIRECT_XY     (RAM_REG + 0x18C) /**< 0x30218C */
#define REG_TOUCH_DIRECT_Z1Z2   (RAM_REG + 0x190) /**< 0x302190 */
#define REG_TOUCH_RZ            (RAM_REG + 0x120) /**< 0x302120 */
#define REG_CTOUCH_TOUCH0_XY    (RAM_REG + 0x124) /**< 0x302124 */
#define REG_TOUCH_SCREEN_XY     (RAM_REG + 0x124) /**< 0x302124 */
#define REG_TOUCH_TAG_XY        (RAM_REG + 0x128) /**< 0x302128 */
#define REG_TOUCH_TAG           (RAM_REG + 0x12C) /**< 0x30212C */
#define REG_TOUCH_TAG1_XY       (RAM_REG + 0x130) /**< 0x302130 */
#define REG_TOUCH_TAG1          (RAM_REG + 0x134) /**< 0x302134 */
#define REG_TOUCH_TAG2_XY       (RAM_REG + 0x138) /**< 0x302138 */
#define REG_TOUCH_TAG2          (RAM_REG + 0x13C) /**< 0x30213C */
#define REG_TOUCH_TAG3_XY       (RAM_REG + 0x140) /**< 0x302140 */
#define REG_TOUCH_TAG3          (RAM_REG + 0x144) /**< 0x302144 */
#define REG_TOUCH_TAG4_XY       (RAM_REG + 0x148) /**< 0x302148 */
#define REG_TOUCH_TAG4          (RAM_REG + 0x14C) /**< 0x30214C */
#define REG_CTOUCH_TOUCH2_XY    (RAM_REG + 0x18C) /**< 0x30218C */
#define REG_CTOUCH_TOUCH3_XY    (RAM_REG + 0x190) /**< 0x302190 */
#define REG_CTOUCH_TOUCH4_X     (RAM_REG + 0x16C) /**< 0x30216C */
#define REG_CTOUCH_TOUCH4_Y     (RAM_REG + 0x120) /**< 0x302120 */
///@}

/**@name Coprocessor Engine Registers */
///@{
#define REG_CMD_READ            (RAM_REG + 0xF8) /**< 0x3020F8 */
#define REG_CMD_WRITE           (RAM_REG + 0xFC) /**< 0x3020FC */
#define REG_CMD_DL              (RAM_REG + 0x100) /**< 0x302100 */
#define REG_CMDB_SPACE          (RAM_REG + 0x574) /**< 0x302574 */
#define REG_CMDB_WRITE          (RAM_REG + 0x578) /**< 0x302578 */
///@}

/**@name Miscellaneous Engine Registers */
///@{
#define REG_ID                  (RAM_REG + 0x0) /**< 0x3020000 */
#define REG_FRAMES              (RAM_REG + 0x4) /**< 0x302004 */
#define REG_CLOCK               (RAM_REG + 0x8) /**< 0x302008 */
#define REG_FREQUENCY           (RAM_REG + 0xC) /**< 0x30200C */
#define REG_CPURESET            (RAM_REG + 0x20) /**< 0x302020 */
#define REG_GPIO_DIR            (RAM_REG + 0x90) /**< 0x302090 */
#define REG_GPIO                (RAM_REG + 0x94) /**< 0x302094 */
#define REG_GPIOX_DIR           (RAM_REG + 0x98) /**< 0x302098 */
#define REG_GPIOX               (RAM_REG + 0x9C) /**< 0x30209C */
#define REG_INT_FLAGS           (RAM_REG + 0xA8) /**< 0x3020A8 */
#define REG_INT_EN              (RAM_REG + 0xAC) /**< 0x3020AC */
#define REG_INT_MASK            (RAM_REG + 0xB0) /**< 0x3020B0 */
#define REG_PWM_HZ              (RAM_REG + 0xD0) /**< 0x3020D0 */
#define REG_PWM_DUTY            (RAM_REG + 0xD4) /**< 0x3020D4 */
#define REG_MACRO_0             (RAM_REG + 0xD8) /**< 0x3020D8 */
#define REG_MACRO_1             (RAM_REG + 0xDC) /**< 0x3020DC */
#define REG_SPI_WIDTH           (RAM_REG + 0x188) /**< 0x302188 */
#define REG_ADAPTIVE_FRAMERATE  (RAM_REG + 0x57C) /**< 0x30257C */
#define REG_UNDERRUN            (RAM_REG + 0x60C) /**< 0x30260C */
#define REG_AH_HCYCLE_MAX       (RAM_REG + 0x610) /**< 0x302610 */
#define REG_PCLK_FREQ           (RAM_REG + 0x614) /**< 0x302614 */
#define REG_PCLK_2X             (RAM_REG + 0x618) /**< 0x302618 */
///@}

/**@name Special Registers */
///@{
#define REG_TRACKER             (RAM_REG + 0x7000) /**< 0x309000 */
#define REG_TRACKER_1           (RAM_REG + 0x7004) /**< 0x309004 */
#define REG_TRACKER_2           (RAM_REG + 0x7008) /**< 0x309008 */
#define REG_TRACKER_3           (RAM_REG + 0x700C) /**< 0x30900C */
#define REG_TRACKER_4           (RAM_REG + 0x7010) /**< 0x309010 */
#define REG_MEDIAFIFO_READ      (RAM_REG + 0x7014) /**< 0x309014 */
#define REG_MEDIAFIFO_WRITE     (RAM_REG + 0x7018) /**< 0x309018 */
#define REG_PLAY_CONTROL        (RAM_REG + 0x714E) /**< 0x30914E */
#define REG_ANIM_ACTIVE         (RAM_REG + 0x702C) /**< 0x30902C */
#define REG_COPRO_PATCH_PTR     (RAM_REG + 0x7162) /**< 0x309162 */
///@}

// clang-format off

/** @name Commands for BT817 */
///@{
#define CMD_DLSTART          4294967040UL /**< 0xFFFFFF00 */
#define CMD_SWAP             4294967041UL /**< 0xFFFFFF01 */
#define CMD_INTERRUPT        4294967042UL /**< 0xFFFFFF02 */
#define CMD_BGCOLOR          4294967049UL /**< 0xFFFFFF09 */
#define CMD_FGCOLOR          4294967050UL /**< 0xFFFFFF0A */
#define CMD_GRADIENT         4294967051UL /**< 0xFFFFFF0B */
#define CMD_TEXT             4294967052UL /**< 0xFFFFFF0C */
#define CMD_BUTTON           4294967053UL /**< 0xFFFFFF0D */
#define CMD_KEYS             4294967054UL /**< 0xFFFFFF0E */
#define CMD_PROGRESS         4294967055UL /**< 0xFFFFFF0F */
#define CMD_SLIDER           4294967056UL /**< 0xFFFFFF10 */
#define CMD_SCROLLBAR        4294967057UL /**< 0xFFFFFF11 */
#define CMD_TOGGLE           4294967058UL /**< 0xFFFFFF12 */
#define CMD_GAUGE            4294967059UL /**< 0xFFFFFF13 */
#define CMD_CLOCK            4294967060UL /**< 0xFFFFFF14 */
#define CMD_CALIBRATE        4294967061UL /**< 0xFFFFFF15 */
#define CMD_SPINNER          4294967062UL /**< 0xFFFFFF16 */
#define CMD_STOP             4294967063UL /**< 0xFFFFFF17 */
#define CMD_MEMCRC           4294967064UL /**< 0xFFFFFF18 */
#define CMD_REGREAD          4294967065UL /**< 0xFFFFFF19 */
#define CMD_MEMWRITE         4294967066UL /**< 0xFFFFFF1A */
#define CMD_MEMSET           4294967067UL /**< 0xFFFFFF1B */
#define CMD_MEMZERO          4294967068UL /**< 0xFFFFFF1C */
#define CMD_MEMCPY           4294967069UL /**< 0xFFFFFF1D */
#define CMD_APPEND           4294967070UL /**< 0xFFFFFF1E */
#define CMD_SNAPSHOT         4294967071UL /**< 0xFFFFFF1F */
#define CMD_BITMAP_TRANSFORM 4294967073UL /**< 0xFFFFFF21 */
#define CMD_INFLATE          4294967074UL /**< 0xFFFFFF22 */
#define CMD_GETPTR           4294967075UL /**< 0xFFFFFF23 */
#define CMD_LOADIMAGE        4294967076UL /**< 0xFFFFFF24 */
#define CMD_GETPROPS         4294967077UL /**< 0xFFFFFF25 */
#define CMD_LOADIDENTITY     4294967078UL /**< 0xFFFFFF26 */
#define CMD_TRANSLATE        4294967079UL /**< 0xFFFFFF27 */
#define CMD_SCALE            4294967080UL /**< 0xFFFFFF28 */
#define CMD_ROTATE           4294967081UL /**< 0xFFFFFF29 */
#define CMD_SETMATRIX        4294967082UL /**< 0xFFFFFF2A */
#define CMD_SETFONT          4294967083UL /**< 0xFFFFFF2B */
#define CMD_TRACK            4294967084UL /**< 0xFFFFFF2C */
#define CMD_DIAL             4294967085UL /**< 0xFFFFFF2D */
#define CMD_NUMBER           4294967086UL /**< 0xFFFFFF2E */
#define CMD_SCREENSAVER      4294967087UL /**< 0xFFFFFF2F */
#define CMD_SKETCH           4294967088UL /**< 0xFFFFFF30 */
#define CMD_LOGO             4294967089UL /**< 0xFFFFFF31 */
#define CMD_COLDSTART        4294967090UL /**< 0xFFFFFF32 */
#define CMD_GETMATRIX        4294967091UL /**< 0xFFFFFF33 */
#define CMD_GRADCOLOR        4294967092UL /**< 0xFFFFFF34 */
#define CMD_SETROTATE        4294967094UL /**< 0xFFFFFF36 */
#define CMD_SNAPSHOT2        4294967095UL /**< 0xFFFFFF37 */
#define CMD_SETBASE          4294967096UL /**< 0xFFFFFF38 */
#define CMD_MEDIAFIFO        4294967097UL /**< 0xFFFFFF39 */
#define CMD_PLAYVIDEO        4294967098UL /**< 0xFFFFFF3A */
#define CMD_SETFONT2         4294967099UL /**< 0xFFFFFF3B */
#define CMD_SETSCRATCH       4294967100UL /**< 0xFFFFFF3C */
#define CMD_ROMFONT          4294967103UL /**< 0xFFFFFF3F */
#define CMD_VIDEOSTART       4294967104UL /**< 0xFFFFFF40 */
#define CMD_VIDEOFRAME       4294967105UL /**< 0xFFFFFF41 */
#define CMD_SYNC             4294967106UL /**< 0xFFFFFF42 */
#define CMD_SETBITMAP        4294967107UL /**< 0xFFFFFF43 */
#define CMD_FLASHERASE       4294967108UL /**< 0xFFFFFF44 */
#define CMD_FLASHWRITE       4294967109UL /**< 0xFFFFFF45 */
#define CMD_FLASHREAD        4294967110UL /**< 0xFFFFFF46 */
#define CMD_FLASHUPDATE      4294967111UL /**< 0xFFFFFF47 */
#define CMD_FLASHDETACH      4294967112UL /**< 0xFFFFFF48 */
#define CMD_FLASHATTACH      4294967113UL /**< 0xFFFFFF49 */
#define CMD_FLASHFAST        4294967114UL /**< 0xFFFFFF4A */
#define CMD_FLASHSPIDESEL    4294967115UL /**< 0xFFFFFF4B */
#define CMD_FLASHSPITX       4294967116UL /**< 0xFFFFFF4C */
#define CMD_FLASHSPIRX       4294967117UL /**< 0xFFFFFF4D */
#define CMD_FLASHSOURCE      4294967118UL /**< 0xFFFFFF4E */
#define CMD_CLEARCACHE       4294967119UL /**< 0xFFFFFF4F */
#define CMD_INFLATE2         4294967120UL /**< 0xFFFFFF50 */
#define CMD_ROTATEAROUND     4294967121UL /**< 0xFFFFFF51 */
#define CMD_RESETFONTS       4294967122UL /**< 0xFFFFFF52 */
#define CMD_ANIMSTART        4294967123UL /**< 0xFFFFFF53 */
#define CMD_ANIMSTOP         4294967124UL /**< 0xFFFFFF54 */
#define CMD_ANIMXY           4294967125UL /**< 0xFFFFFF55 */
#define CMD_ANIMDRAW         4294967126UL /**< 0xFFFFFF56 */
#define CMD_GRADIENTA        4294967127UL /**< 0xFFFFFF57 */
#define CMD_FILLWIDTH        4294967128UL /**< 0xFFFFFF58 */
#define CMD_APPENDF          4294967129UL /**< 0xFFFFFF59 */
#define CMD_ANIMFRAME        4294967130UL /**< 0xFFFFFF5A */
#define CMD_NOP              4294967131UL /**< 0xFFFFFF5B */
#define CMD_VIDEOSTARTF      4294967135UL /**< 0xFFFFFF5F */
#define CMD_CALIBRATESUB     4294967136UL /**< 0xFFFFFF60 */
#define CMD_TESTCARD         4294967137UL /**< 0xFFFFFF61 */
#define CMD_HSF              4294967138UL /**< 0xFFFFFF62 */
#define CMD_APILEVEL         4294967139UL /**< 0xFFFFFF63 */
#define CMD_GETIMAGE         4294967140UL /**< 0xFFFFFF64 */
#define CMD_WAIT             4294967141UL /**< 0xFFFFFF65 */
#define CMD_RETURN           4294967142UL /**< 0xFFFFFF66 */
#define CMD_CALLLIST         4294967143UL /**< 0xFFFFFF67 */
#define CMD_NEWLIST          4294967144UL /**< 0xFFFFFF68 */
#define CMD_ENDLIST          4294967145UL /**< 0xFFFFFF69 */
#define CMD_PCLKFREQ         4294967146UL /**< 0xFFFFFF6A */
#define CMD_FONTCACHE        4294967147UL /**< 0xFFFFFF6B */
#define CMD_FONTCACHEQUERY   4294967148UL /**< 0xFFFFFF6C */
#define CMD_ANIMFRAMERAM     4294967149UL /**< 0xFFFFFF6D */
#define CMD_ANIMSTARTRAM     4294967150UL /**< 0xFFFFFF6E */
#define CMD_RUNANIM          4294967151UL /**< 0xFFFFFF6F */
#define CMD_FLASHPROGRAM     4294967152UL /**< 0xFFFFFF70 */

// clang-format on
///@}

#define SCISSOR_XY_SHIFT 11
#define SCISSOR_XY_MASK 2047UL
#define SCISSOR_SIZE_SHIFT 12
#define SCISSOR_SIZE_MASK 4095UL

#define BITMAP_ADDR_MASK 16777215UL /**< 0xFFFFFF */

/** @name Display List */
///@{

#define VERTEX2F(x, y)                      ((1UL << 30) | (((x)&32767UL) << 15) | (((y)&32767UL) << 0))
#define VERTEX2II(x, y, handle, cell)       ((2UL << 30) | (((x)&511UL) << 21) | (((y)&511UL) << 12) | (((handle)&31UL) << 7) | (((cell)&127UL) << 0))
#define DISPLAY()                           ((0UL << 24))
#define BITMAP_SOURCE(addr)                 ((1UL << 24) | ((addr) < 0 ? (((addr) & (BITMAP_ADDR_MASK >> 1))) : ((addr)&BITMAP_ADDR_MASK)))
#define BITMAP_SOURCE2(flash_or_ram, addr)  ((1UL << 24) | ((flash_or_ram) << 23) | (((addr) & (BITMAP_ADDR_MASK >> 1)) << 0))
#define CLEAR_COLOR_RGB(red, green, blue)   ((2UL << 24) | (((red)&255UL) << 16) | (((green)&255UL) << 8) | (((blue)&255UL) << 0))
#define TAG(s)                              ((3UL << 24) | (((s)&255UL) << 0))
#define COLOR_RGB(red, green, blue)         ((4UL << 24) | (((red)&255UL) << 16) | (((green)&255UL) << 8) | (((blue)&255UL) << 0))
#define BITMAP_HANDLE(handle)               ((5UL << 24) | (((handle)&31UL) << 0))
#define CELL(cell)                          ((6UL << 24) | (((cell)&127UL) << 0))
#define BITMAP_LAYOUT(format, linestride, height)        ((7UL << 24) | (((format)&31UL) << 19) | (((linestride)&1023UL) << 9) | (((height)&511UL) << 0))
#define BITMAP_SIZE(filter, wrapx, wrapy, width, height) ((8UL << 24) | (((filter)&1UL) << 20) | (((wrapx)&1UL) << 19) | (((wrapy)&1UL) << 18) | (((width)&511UL) << 9) | (((height)&511UL) << 0))
#define ALPHA_FUNC(func, ref)               ((9UL << 24) | (((func)&7UL) << 8) | (((ref)&255UL) << 0))
#define STENCIL_FUNC(func, ref, mask)       ((10UL << 24) | (((func)&7UL) << 16) | (((ref)&255UL) << 8) | (((mask)&255UL) << 0))
#define BLEND_FUNC(src, dst)                ((11UL << 24) | (((src)&7UL) << 3) | (((dst)&7UL) << 0))
#define STENCIL_OP(sfail, spass)            ((12UL << 24) | (((sfail)&7UL) << 3) | (((spass)&7UL) << 0))
#define POINT_SIZE(size)                    ((13UL << 24) | (((size)&8191UL) << 0))
#define LINE_WIDTH(width)                   ((14UL << 24) | (((width)&4095UL) << 0))
#define CLEAR_COLOR_A(alpha)                ((15UL << 24) | (((alpha)&255UL) << 0))
#define COLOR_A(alpha)                      ((16UL << 24) | (((alpha)&255UL) << 0))
#define CLEAR_STENCIL(s)                    ((17UL << 24) | (((s)&255UL) << 0))
#define CLEAR_TAG(s)                        ((18UL << 24) | (((s)&255UL) << 0))
#define STENCIL_MASK(mask)                  ((19UL << 24) | (((mask)&255UL) << 0))
#define TAG_MASK(mask)                      ((20UL << 24) | (((mask)&1UL) << 0))
#define BITMAP_TRANSFORM_A_EXT(p, v)        ((21UL << 24) | (((p)&1UL) << 17) | (((v)&131071UL) << 0))
#define BITMAP_TRANSFORM_B_EXT(p, v)        ((22UL << 24) | (((p)&1UL) << 17) | (((v)&131071UL) << 0))
#define BITMAP_TRANSFORM_D_EXT(p, v)        ((24UL << 24) | (((p)&1UL) << 17) | (((v)&131071UL) << 0))
#define BITMAP_TRANSFORM_E_EXT(p, v)        ((25UL << 24) | (((p)&1UL) << 17) | (((v)&131071UL) << 0))
#define BITMAP_TRANSFORM_A(a)               BITMAP_TRANSFORM_A_EXT(0, a)
#define BITMAP_TRANSFORM_B(b)               BITMAP_TRANSFORM_B_EXT(0, b)
#define BITMAP_TRANSFORM_D(d)               BITMAP_TRANSFORM_D_EXT(0, d)
#define BITMAP_TRANSFORM_E(e)               BITMAP_TRANSFORM_E_EXT(0, e)
#define BITMAP_TRANSFORM_C(c)               ((23UL << 24) | (((c)&16777215UL) << 0))
#define BITMAP_TRANSFORM_F(f)               ((26UL << 24) | (((f)&16777215UL) << 0))
#define SCISSOR_XY(x, y)                    ((27UL << 24) | (((x)&SCISSOR_XY_MASK) << SCISSOR_XY_SHIFT) | (((y)&SCISSOR_XY_MASK) << 0))
#define SCISSOR_SIZE(width, height)         ((28UL << 24) | (((width)&SCISSOR_SIZE_MASK) << SCISSOR_SIZE_SHIFT) | (((height)&SCISSOR_SIZE_MASK) << 0))
#define CALL(dest)                          ((29UL << 24) | (((dest)&65535UL) << 0))
#define JUMP(dest)                          ((30UL << 24) | (((dest)&65535UL) << 0))
#define BEGIN(prim)                         ((31UL << 24) | (((prim)&15UL) << 0))
#define COLOR_MASK(r, g, b, a)              ((32UL << 24) | (((r)&1UL) << 3) | (((g)&1UL) << 2) | (((b)&1UL) << 1) | (((a)&1UL) << 0))
#define END()                               ((33UL << 24))
#define SAVE_CONTEXT()                      ((34UL << 24))
#define RESTORE_CONTEXT()                   ((35UL << 24))
#define RETURN()                            ((36UL << 24))
#define MACRO(m)                            ((37UL << 24) | (((m)&1UL) << 0))
#define CLEAR(c, s, t)                      ((38UL << 24) | (((c)&1UL) << 2) | (((s)&1UL) << 1) | (((t)&1UL) << 0))
#define VERTEX_FORMAT(frac)                 ((39UL << 24) | (((frac)&7UL) << 0))
#define BITMAP_LAYOUT_H(linestride, height) ((40UL << 24) | (((linestride)&3UL) << 2) | (((height)&3UL) << 0))
#define BITMAP_SIZE_H(width, height)        ((41UL << 24) | (((width)&3UL) << 2) | (((height)&3UL) << 0))
#define PALETTE_SOURCE(addr)                ((42UL << 24) | (((addr)&4194303UL) << 0))
#define VERTEX_TRANSLATE_X(x)               ((43UL << 24) | (((x)&131071UL) << 0))
#define VERTEX_TRANSLATE_Y(y)               ((44UL << 24) | (((y)&131071UL) << 0))
#define NOP()                               ((45UL << 24))
#define BITMAP_EXT_FORMAT(format)           ((46UL << 24) | (((format)&65535UL) << 0))
#define BITMAP_SWIZZLE(r, g, b, a)          ((47UL << 24) | (((r)&7UL) << 9) | (((g)&7UL) << 6) | (((b)&7UL) << 3) | (((a)&7UL) << 0))
#define INT_FRR()                           ((48UL << 24))
///@}

/** @name Options */
///@{
#define OPT_3D           0UL
#define OPT_RGB565       0UL
#define OPT_MONO         1UL
#define OPT_NODL         2UL
#define OPT_NOTEAR       4UL
#define OPT_FULLSCREEN   8UL
#define OPT_MEDIAFIFO    16UL
#define OPT_SOUND        32UL
#define OPT_FLASH        64UL
#define OPT_OVERLAY      128UL
#define OPT_FLAT         256UL
#define OPT_SIGNED       256UL
#define OPT_DITHER       256UL
#define OPT_CENTERX      512UL
#define OPT_CENTERY      1024UL
#define OPT_CENTER       1536UL
#define OPT_RIGHTX       2048UL
#define OPT_FORMAT       4096UL
#define OPT_NOBACK       4096UL
#define OPT_NOTICKS      8192UL
#define OPT_FILL         8192UL
#define OPT_NOHM         16384UL
#define OPT_NOPOINTER    16384UL
#define OPT_NOSECS       32768UL
#define OPT_NOHANDS      49152UL
///@}

/** @name for CMD_ANIMSTART */
///@{
#define ANIM_ONCE        0UL
#define ANIM_LOOP        1UL
#define ANIM_HOLD        2UL
///@}

/** @name for STENCIL_OP */
///@{
#define ZERO             0UL
#define KEEP             1UL
#define REPLACE          2UL
#define INCR             3UL
#define DECR             4UL
#define INVERT           5UL
///@}

/** @name for BLEND_FUNC */
///@{
//#define ZERO 0UL
#define ONE              1UL
#define SRC_ALPHA        2UL
#define DST_ALPHA        3UL
#define ONE_MINUS_SRC_ALPHA 4UL
#define ONE_MINUS_DST_ALPHA 5UL
///@}

/** @name for BITMAP_SWIZZLE */
///@{
//#define ZERO            0UL
//#define ONE             1UL
#define RED              2UL
#define GREEN            3UL
#define BLUE             4UL
#define ALPHA            5UL
///@}

/** @name for ALPHA_FUNC */
///@{
#define NEVER            0UL
#define LESS             1UL
#define LEQUAL           2UL
#define GREATER          3UL
#define GEQUAL           4UL
#define EQUAL            5UL
#define NOTEQUAL         6UL
#define ALWAYS           7UL
///@}

/** @name format of the bitmap for BITMAP_EXT_FORMAT */
///@{
#define ARGB1555         0UL
#define L1               1UL
#define L4               2UL
#define L8               3UL
#define RGB332           4UL
#define ARGB2            5UL
#define ARGB4            6UL
#define RGB565           7UL
#define PALETTED         8UL
#define TEXT8X8          9UL
#define TEXTVGA          10UL
#define BARGRAPH         11UL
#define PALETTED565      14UL
#define PALETTED4444     15UL
#define PALETTED8        16UL
#define L2               17UL
#define GLFORMAT         31UL
#define COMPRESSED_RGBA_ASTC_4x4_KHR   37808UL
#define COMPRESSED_RGBA_ASTC_5x4_KHR   37809UL
#define COMPRESSED_RGBA_ASTC_5x5_KHR   37810UL
#define COMPRESSED_RGBA_ASTC_6x5_KHR   37811UL
#define COMPRESSED_RGBA_ASTC_6x6_KHR   37812UL
#define COMPRESSED_RGBA_ASTC_8x5_KHR   37813UL
#define COMPRESSED_RGBA_ASTC_8x6_KHR   37814UL
#define COMPRESSED_RGBA_ASTC_8x8_KHR   37815UL
#define COMPRESSED_RGBA_ASTC_10x5_KHR  37816UL
#define COMPRESSED_RGBA_ASTC_10x6_KHR  37817UL
#define COMPRESSED_RGBA_ASTC_10x8_KHR  37818UL
#define COMPRESSED_RGBA_ASTC_10x10_KHR 37819UL
#define COMPRESSED_RGBA_ASTC_12x10_KHR 37820UL
#define COMPRESSED_RGBA_ASTC_12x12_KHR 37821UL
///@}

/** @name for BITMAP_SIZE */
///@{
//filter
#define NEAREST          0UL
#define BILINEAR         1UL
//wrapx/wrapy
#define BORDER           0UL
#define REPEAT           1UL
///@}

#ifdef POINTS
#undef POINTS
#endif
/** @name for BEGIN */
///@{
#define BITMAPS          1UL
#define POINTS           2UL
#define LINES            3UL
#define LINE_STRIP       4UL
#define EDGE_STRIP_R     5UL
#define EDGE_STRIP_L     6UL
#define EDGE_STRIP_A     7UL
#define EDGE_STRIP_B     8UL
#define RECTS            9UL
///@}

/** @name for REG_LASH_STATUS */
///@{
#define FLASH_STATUS_INIT       0UL
#define FLASH_STATUS_DETACHED   1UL
#define FLASH_STATUS_BASIC      2UL
#define FLASH_STATUS_FULL       3UL
///@}

/**< @name for REG_DLSWAP */
///@{
#define DLSWAP_DONE       0UL
#define DLSWAP_LINE       1UL
#define DLSWAP_FRAME      2UL
///@}

/** @name for REG_PLAYBACK_FORMAT */
///@{
#define LINEAR_SAMPLES    0UL
#define ULAW_SAMPLES      1UL
#define ADPCM_SAMPLES     2UL
///@}

/** @name for REG_TOUCH_MODE */
///@{
#define TOUCHMODE_OFF     0UL
#define TOUCHMODE_ONESHOT 1UL
#define TOUCHMODE_FRAME   2UL
#define TOUCHMODE_CONTINUOUS 3UL
///@}

/** @name for REG_TOUCH_ADC_MODE */
///@{
#define ADC_SINGLE_ENDED  0UL
#define ADC_DIFFERENTIAL  1UL
///@}

/** @name for REG_CTOUCH_EXTENDED */
///@{
#define CTOUCH_MODE_COMPATIBILITY 1UL
#define CTOUCH_MODE_EXTENDED      0UL
///@}

/** @name for REG_INT_FLAGS */
///@{
#define INT_SWAP          1UL
#define INT_TOUCH         2UL
#define INT_TAG           4UL
#define INT_SOUND         8UL
#define INT_L8C           12UL
#define INT_VGA           13UL
#define INT_G8            18UL
#define INT_PLAYBACK      16UL
#define INT_CMDEMPTY      32UL
#define INT_CMDFLAG       64UL
#define INT_CONVCOMPLETE  128UL
#define INT_UNDERRUN      256UL
///@}

#endif /* #ifndef EVE_GPU_DEFS__H */

/* end of file */
