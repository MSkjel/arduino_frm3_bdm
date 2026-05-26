// SPDX-License-Identifier: MIT
//
// arduino_frm3_bdm: BDM firmware for the MC9S12XEQ384.
//
// Runs on an Arduino Uno R4 (WiFi or Minima). Bit-bangs the HCS12X BDM
// serial protocol on D2 (BKGD) and D3 (RESET). Exposes a line-oriented
// text command interface over USB CDC at 1 Mbaud, consumed by the
// Python host driver in frm3.py.
//
// Wiring: D2 to BKGD test pad, D3 to RESET test pad, both through
// 1 kOhm series resistors. GND between Arduino and FRM3. Power the
// FRM3 from its own supply.
//
// Address spaces use the chip's 23-bit global address:
//   P-Flash:        0x780000..0x79FFFF (block 1)
//                   0x7C0000..0x7FFFFF (block 2)
//   D-Flash:        0x100000..0x107FFF
//   EEE buffer RAM: 0x13F000..0x13FFFF (via BDMGPR + local 0xF000)
//
// Send `help` over serial for the command list.

#include <Arduino.h>

// Pin / port mapping (RA4M1 PORT1 direct register access).
// The D2/D3 to P10x mapping differs between Uno R4 board variants:
//   WiFi   : D2 = P104, D3 = P105
//   Minima : D2 = P105, D3 = P104
#if defined(ARDUINO_UNOR4_WIFI) || defined(ARDUINO_UNOWIFIR4)
constexpr uint8_t BKGD_BIT = 4;
constexpr uint8_t RST_BIT = 5;
#else
constexpr uint8_t BKGD_BIT = 5;
constexpr uint8_t RST_BIT = 4;
#endif
constexpr uint8_t BKGD_PIN = 2;
constexpr uint8_t RESET_PIN = 3;

#define PORT_SET_HIGH(bit) (R_PORT1->PCNTR3 = (1u << (bit)))
#define PORT_SET_LOW(bit) (R_PORT1->PCNTR3 = (1u << ((bit) + 16)))
#define PORT_SET_OUT(bit) (R_PORT1->PCNTR1 |= (1u << (bit)))
#define PORT_SET_IN(bit) (R_PORT1->PCNTR1 &= ~(1u << (bit)))
#define PORT_READ(bit) ((R_PORT1->PCNTR2 >> (bit)) & 1u)

#define BKGD_HIGH() PORT_SET_HIGH(BKGD_BIT)
#define BKGD_LOW() PORT_SET_LOW(BKGD_BIT)
#define BKGD_OUT() PORT_SET_OUT(BKGD_BIT)
#define BKGD_IN() PORT_SET_IN(BKGD_BIT)
#define BKGD_READ() PORT_READ(BKGD_BIT)
#define RST_HIGH() PORT_SET_HIGH(RST_BIT)
#define RST_LOW() PORT_SET_LOW(RST_BIT)
#define RST_OUT() PORT_SET_OUT(RST_BIT)
#define RST_IN() PORT_SET_IN(RST_BIT)

// HCS12X BDM hardware command opcodes (RM section 4).
constexpr uint8_t BDM_ACK_ENABLE = 0xD5;
constexpr uint8_t BDM_READ_BYTE = 0xE0;
constexpr uint8_t BDM_READ_WORD = 0xE8;
constexpr uint8_t BDM_WRITE_BYTE = 0xC0;
constexpr uint8_t BDM_WRITE_WORD = 0xC8;
constexpr uint8_t BDM_READ_BD_BYTE = 0xE4; // "in map" - accesses BDM register space
constexpr uint8_t BDM_WRITE_BD_BYTE = 0xC4;

// CPU debug commands (RM section 7.4.5 / 7.4.7).
// BACKGROUND is a hardware command and works any time ENBDM=1. The
// READ_* firmware commands only work once the CPU is halted in
// active BDM.
constexpr uint8_t BDM_BACKGROUND = 0x90;
constexpr uint8_t BDM_GO = 0x08;
constexpr uint8_t BDM_TRACE1 = 0x10;
constexpr uint8_t BDM_READ_NEXT = 0x62;
constexpr uint8_t BDM_READ_PC = 0x63;
constexpr uint8_t BDM_READ_D = 0x64;
constexpr uint8_t BDM_READ_X = 0x65;
constexpr uint8_t BDM_READ_Y = 0x66;
constexpr uint8_t BDM_READ_SP = 0x67;
constexpr uint8_t BDM_WRITE_PC = 0x43;
constexpr uint16_t BDMSTS_BD_ADDR = 0xFF01; // ENBDM bit 7, BDMACT bit 6

// MC9S12XE flash module registers (RM §26.3)
constexpr uint16_t FCLKDIV_REG = 0x0100;
constexpr uint16_t FSEC_REG = 0x0101;
constexpr uint16_t FCCOBIX_REG = 0x0102;
constexpr uint16_t FCNFG_REG = 0x0104;
constexpr uint16_t FSTAT_REG = 0x0106;
constexpr uint16_t FERSTAT_REG = 0x0107;
constexpr uint16_t FPROT_REG = 0x0108;
constexpr uint16_t DFPROT_REG = 0x0109;
constexpr uint16_t FCCOBHI_REG = 0x010A;
constexpr uint16_t FCCOBLO_REG = 0x010B;
constexpr uint16_t EPAGE_REG = 0x0017;
constexpr uint16_t PPAGE_REG = 0x0015;
constexpr uint16_t COPCTL_REG = 0x003C;

// FSTAT bits
constexpr uint8_t FSTAT_CCIF = 0x80;
constexpr uint8_t FSTAT_ACCERR = 0x20;
constexpr uint8_t FSTAT_FPVIOL = 0x10;

// FCMD opcodes (S12XFTM384K2V1, RM §26.4.2)
constexpr uint8_t FCMD_PROGRAM_P_FLASH = 0x06;
constexpr uint8_t FCMD_ERASE_P_FLASH_SECTOR = 0x0A;
constexpr uint8_t FCMD_FULL_PARTITION_D = 0x0F;
constexpr uint8_t FCMD_PROGRAM_D_FLASH = 0x11;
constexpr uint8_t FCMD_ERASE_D_SECTOR = 0x12;
constexpr uint8_t FCMD_ENABLE_EEE = 0x13;
constexpr uint8_t FCMD_DISABLE_EEE = 0x14;
constexpr uint8_t FCMD_EEE_QUERY = 0x15;

// EEE buffer RAM lives at 0x13_F000-0x13_FFFF (4 KB) on the 384K module.
// Access from BDM uses BDMGPR with BGAE=1 + 16-bit local 0xF000-0xFFFF.
constexpr uint8_t EEE_BUF_GLOBAL_HI = 0x13;     // top 7 bits of global addr
constexpr uint16_t EEE_BUF_LOCAL_BASE = 0xF000; // local addr when BGAE=1
constexpr uint32_t EEE_BUF_SIZE = 0x1000;       // 4 KB
constexpr uint16_t BDMGPR_BD_ADDR = 0xFF08;     // BDM-space register

// FSEC sector (last 1 KB of P-Flash). Erasing it would re-secure the chip.
constexpr uint32_t FSEC_SECTOR_BASE = 0x7FFC00;

// BDM timing state, populated by bdm_sync().
struct BdmState
{
    bool synced;
    uint32_t bus_period_ns;
    uint32_t bit_period_ns;
    uint32_t tx1_low_ns;
    uint32_t tx0_low_ns;
    uint32_t rx_low_ns;
    uint32_t rx_sample_ns;
    bool ack_enabled;
};
static BdmState bdm = {false, 0, 0, 0, 0, 0, 0, false};

// DWT cycle counter, 48 MHz tick. Provides sub-microsecond timing for
// the bit-bang routines below.
static void dwt_init()
{
    *((volatile uint32_t *)0xE000EDFC) |= (1u << 24);
    *((volatile uint32_t *)0xE0001000) |= 1u;
}
static inline uint32_t dwt_cycles() { return *((volatile uint32_t *)0xE0001004); }
static inline void wait_until(uint32_t target)
{
    while ((int32_t)(dwt_cycles() - target) < 0)
    { /* spin */
    }
}
static inline uint32_t ns_to_cyc(uint32_t ns)
{
    return (ns * 48u + 500u) / 1000u;
}

// SYNC: pulse BKGD low and measure the chip's 128-cycle response.
// The pulse width divided by 128 gives the chip's bus period, which
// every other bit-bang timing is derived from. Uses DWT cycle counter
// rather than micros() because the response pulse is only tens of us
// wide and micros() resolution is too coarse.
static bool bdm_sync()
{
    bdm.synced = false;
    BKGD_IN();
    delayMicroseconds(50);
    BKGD_LOW();
    BKGD_OUT();
    delayMicroseconds(200);
    BKGD_IN();
    // Wait for chip to release BKGD high
    uint32_t t0 = dwt_cycles();
    const uint32_t TO_10MS = 48u * 10000u;
    const uint32_t TO_50MS = 48u * 50000u;
    while (BKGD_READ() == 0)
    {
        if (dwt_cycles() - t0 > TO_10MS)
            return false;
    }
    // Wait for chip to drive BKGD low (start of 128-cycle response)
    t0 = dwt_cycles();
    while (BKGD_READ() == 1)
    {
        if (dwt_cycles() - t0 > TO_50MS)
            return false;
    }
    uint32_t pulse_start = dwt_cycles();
    // Wait for the low pulse to end
    while (BKGD_READ() == 0)
    {
        if (dwt_cycles() - pulse_start > TO_50MS)
            return false;
    }
    uint32_t pulse_cyc = dwt_cycles() - pulse_start;
    if (pulse_cyc == 0)
        return false;
    // pulse_cyc is DWT cycles at 48 MHz. Convert to ns: cyc * 1000 / 48.
    // Then divide by 128 to get per-target-cycle (chip bus period).
    // Do both in one go: bus_period_ns = pulse_cyc * 1000 / (48 * 128)
    //                                  = pulse_cyc * 1000 / 6144
    bdm.bus_period_ns = (pulse_cyc * 1000u + 3072u) / 6144u; // +half divisor for rounding
    // Spec §7.4.6: 16 target cycles per bit. 32 = 2× spec margin.
    bdm.bit_period_ns = bdm.bus_period_ns * 32u;
    bdm.tx1_low_ns = bdm.bus_period_ns * 4u;
    bdm.tx0_low_ns = bdm.bus_period_ns * 13u;
    bdm.rx_low_ns = bdm.bus_period_ns * 4u;
    bdm.rx_sample_ns = bdm.bus_period_ns * 10u;
    bdm.synced = true;
    return true;
}

// Bit/byte transmit and receive. Must run with IRQs disabled.
static inline void bdm_send_bit(uint8_t bit)
{
    const uint32_t low_c = ns_to_cyc(bit ? bdm.tx1_low_ns : bdm.tx0_low_ns);
    const uint32_t bit_c = ns_to_cyc(bdm.bit_period_ns);
    BKGD_LOW();
    BKGD_OUT();
    const uint32_t t = dwt_cycles();
    wait_until(t + low_c);
    BKGD_HIGH();
    BKGD_IN();
    wait_until(t + bit_c);
}

static inline uint8_t bdm_recv_bit()
{
    const uint32_t lo_c = ns_to_cyc(bdm.rx_low_ns);
    const uint32_t smp_c = ns_to_cyc(bdm.rx_sample_ns);
    const uint32_t bit_c = ns_to_cyc(bdm.bit_period_ns);
    BKGD_LOW();
    BKGD_OUT();
    const uint32_t t = dwt_cycles();
    wait_until(t + lo_c);
    BKGD_IN();
    wait_until(t + smp_c);
    uint8_t v = BKGD_READ();
    wait_until(t + bit_c);
    return v;
}

static void bdm_send_byte(uint8_t b)
{
    for (int i = 7; i >= 0; i--)
        bdm_send_bit((b >> i) & 1);
}
static uint8_t bdm_recv_byte()
{
    uint8_t b = 0;
    for (int i = 7; i >= 0; i--)
        b |= bdm_recv_bit() << i;
    return b;
}
static void bdm_send_word(uint16_t w)
{
    bdm_send_byte(w >> 8);
    bdm_send_byte(w & 0xFF);
}
static uint16_t bdm_recv_word()
{
    uint16_t hi = bdm_recv_byte();
    return (hi << 8) | bdm_recv_byte();
}

// ACK handshake. The target drives BKGD low for ~16 cycles after each
// command (RM 4.7). Must run with IRQs disabled.
static bool bdm_wait_ack(uint32_t timeout_us)
{
    BKGD_IN();
    const uint32_t timeout_cyc = timeout_us * 48u;
    uint32_t t0 = dwt_cycles();
    while (BKGD_READ() != 0)
    {
        if ((dwt_cycles() - t0) > timeout_cyc)
            return false;
    }
    t0 = dwt_cycles();
    while (BKGD_READ() == 0)
    {
        if ((dwt_cycles() - t0) > timeout_cyc)
            return false;
    }
    uint32_t s0 = dwt_cycles();
    while ((dwt_cycles() - s0) < 100u)
    {
    }
    return true;
}

// SYNC abort: drive BKGD low for >128 target cycles to force the chip's
// BDM state machine back to idle. Per RM 4.7 this also auto-disables
// ACK on the target, so callers must re-enable.
static void bdm_sync_abort()
{
    BKGD_LOW();
    BKGD_OUT();
    delayMicroseconds((bdm.bus_period_ns * 200u) / 1000u + 5u);
    BKGD_HIGH();
    BKGD_IN();
    delayMicroseconds((bdm.bus_period_ns * 300u) / 1000u + 10u);
    bdm.ack_enabled = false;
}

static bool bdm_enable_ack_inner()
{
    bdm.ack_enabled = false;
    bdm_send_byte(BDM_ACK_ENABLE);
    bool ok = bdm_wait_ack(50000);
    bdm.ack_enabled = ok;
    return ok;
}

// Post-command wait. On ACK timeout, run a SYNC abort and re-enable
// ACK so the next command can still use the handshake; without this
// the BDM stays stuck and subsequent reads return 0xFF.
static bool bdm_post_ack()
{
    if (!bdm.ack_enabled)
    {
        delayMicroseconds((bdm.bus_period_ns * 150u) / 1000u + 1u);
        return true;
    }
    if (bdm_wait_ack(5000))
        return true;
    bdm_sync_abort();
    for (uint8_t a = 0; a < 3; a++)
    {
        if (bdm_enable_ack_inner())
            break;
        delayMicroseconds(500);
    }
    return false;
}

static bool bdm_enable_ack()
{
    __disable_irq();
    bool ok = bdm_enable_ack_inner();
    __enable_irq();
    return ok;
}

// BDM read / write primitives. Each retries up to 4 times on ACK timeout.
static bool bdm_last_err = false;

// READ_BD_BYTE reads BDM-space registers (0xFF00..0xFF0F). Same wire
// protocol as READ_BYTE but opcode 0xE4 enables the BDM register
// overlay for this transaction.
static uint8_t bdm_read_bd_byte(uint16_t addr)
{
    for (uint8_t r = 0; r < 4; r++)
    {
        __disable_irq();
        bdm_last_err = false;
        bdm_send_byte(BDM_READ_BD_BYTE);
        bdm_send_word(addr);
        bool ok = bdm_post_ack();
        if (!ok)
        {
            bdm_last_err = true;
            __enable_irq();
            continue;
        }
        uint16_t data = bdm_recv_word();
        __enable_irq();
        return (addr & 1) ? (data & 0xFF) : (data >> 8);
    }
    return 0xFF;
}

static void bdm_write_bd_byte(uint16_t addr, uint8_t val)
{
    for (uint8_t r = 0; r < 4; r++)
    {
        __disable_irq();
        bdm_last_err = false;
        bdm_send_byte(BDM_WRITE_BD_BYTE);
        bdm_send_word(addr);
        bdm_send_byte(val);
        bdm_send_byte(val);
        bool ok = bdm_post_ack();
        __enable_irq();
        if (ok)
            return;
        bdm_last_err = true;
    }
}

static uint8_t bdm_read_byte(uint16_t addr)
{
    for (uint8_t r = 0; r < 4; r++)
    {
        __disable_irq();
        bdm_last_err = false;
        bdm_send_byte(BDM_READ_BYTE);
        bdm_send_word(addr);
        bool ok = bdm_post_ack();
        if (!ok)
        {
            bdm_last_err = true;
            __enable_irq();
            continue;
        }
        uint16_t data = bdm_recv_word();
        __enable_irq();
        return (addr & 1) ? (data & 0xFF) : (data >> 8);
    }
    return 0xFF;
}

static void bdm_write_byte(uint16_t addr, uint8_t val)
{
    for (uint8_t r = 0; r < 4; r++)
    {
        __disable_irq();
        bdm_last_err = false;
        bdm_send_byte(BDM_WRITE_BYTE);
        bdm_send_word(addr);
        bdm_send_byte(val);
        bdm_send_byte(val); // protocol mirrors the byte in the 16-bit field
        bool ok = bdm_post_ack();
        __enable_irq();
        if (ok)
            return;
        bdm_last_err = true;
    }
}

static uint16_t bdm_read_word(uint16_t addr)
{
    for (uint8_t r = 0; r < 4; r++)
    {
        __disable_irq();
        bdm_last_err = false;
        bdm_send_byte(BDM_READ_WORD);
        bdm_send_word(addr & 0xFFFE);
        bool ok = bdm_post_ack();
        if (!ok)
        {
            bdm_last_err = true;
            __enable_irq();
            continue;
        }
        uint16_t w = bdm_recv_word();
        __enable_irq();
        return w;
    }
    return 0xFFFF;
}

static void bdm_write_word(uint16_t addr, uint16_t val)
{
    for (uint8_t r = 0; r < 4; r++)
    {
        __disable_irq();
        bdm_last_err = false;
        bdm_send_byte(BDM_WRITE_WORD);
        bdm_send_word(addr & 0xFFFE);
        bdm_send_word(val);
        bool ok = bdm_post_ack();
        __enable_irq();
        if (ok)
            return;
        bdm_last_err = true;
    }
}

// CPU register write via BDM firmware command. Opcode + 16-bit value, ACK.
static void bdm_write_cpu_reg(uint8_t opcode, uint16_t val)
{
    for (uint8_t r = 0; r < 4; r++)
    {
        __disable_irq();
        bdm_last_err = false;
        bdm_send_byte(opcode);
        bdm_send_word(val);
        bool ok = bdm_post_ack();
        __enable_irq();
        if (ok)
            return;
        bdm_last_err = true;
    }
}

// CPU register read via BDM firmware command. The opcode has no parameters
// and the chip returns a 16-bit value after an ACK pulse. Only valid when
// the CPU is halted in active BDM (BDMACT=1).
static uint16_t bdm_read_cpu_reg(uint8_t opcode)
{
    for (uint8_t r = 0; r < 4; r++)
    {
        __disable_irq();
        bdm_last_err = false;
        bdm_send_byte(opcode);
        bool ok = bdm_post_ack();
        if (!ok)
        {
            bdm_last_err = true;
            __enable_irq();
            continue;
        }
        uint16_t w = bdm_recv_word();
        __enable_irq();
        return w;
    }
    return 0xFFFF;
}

// Send BACKGROUND opcode (0x90). Hardware command - works when ENBDM=1.
// On success the chip halts the CPU into active BDM (BDMACT=1).
static void bdm_send_background()
{
    __disable_irq();
    bdm_send_byte(BDM_BACKGROUND);
    __enable_irq();
    delayMicroseconds(200); // give the chip time to halt
}

// Reset chip into NORMAL single-chip mode (firmware runs). Unlike
// bdm_enter() which holds BKGD low across the RESET rising edge to
// select active-BDM mode, this lets BKGD idle high so MODA/MODC latch
// "normal single chip" and the chip boots its real firmware.
static uint8_t bdm_reset_normal()
{
    BKGD_IN(); // release BKGD (input, pulled high)
    RST_LOW();
    RST_OUT();
    delayMicroseconds(2000);
    RST_IN(); // release RESET
    delay(50);
    if (PORT_READ(RST_BIT) == 0)
        return 1; // RESET stuck low
    return 0;
}

// Halt a running CPU into active BDM. Requires the BDM serial interface
// to be sync'd (i.e. caller already did bdm_sync). Sequence:
//   1. Set ENBDM=1 in BDMSTS so BACKGROUND will be honoured.
//   2. Send BACKGROUND. CPU completes the current instruction then halts.
//   3. Read BDMSTS back and verify BDMACT=1.
// Returns 0 ok, 1 ENBDM write failed, 2 BDMSTS readback failed, 3 BDMACT
// never set (CPU did not honour the halt).
static uint8_t bdm_halt()
{
    bdm_write_bd_byte(BDMSTS_BD_ADDR, 0x80);
    if (bdm_last_err)
        return 1;
    delayMicroseconds(200);
    bdm_send_background();
    delayMicroseconds(500);
    uint8_t sts = bdm_read_bd_byte(BDMSTS_BD_ADDR);
    if (bdm_last_err)
        return 2;
    if (!(sts & 0x40))
        return 3; // BDMACT bit
    return 0;
}

// Active BDM entry. Drive RESET low and BKGD low, then release RESET.
// The chip latches MODA/MODC at the RESET rising edge; BKGD low at
// that moment selects special-single-chip mode (active BDM).
// Returns 0 ok, 1 RESET stuck, 2 BKGD not idle high, 3 SYNC failed.
static uint8_t bdm_enter_rst_after_release = 0xFF;
static uint8_t bdm_enter_bkgd_after_release = 0xFF;

static uint8_t bdm_enter()
{
    // Retry up to 3 times. If the chip is already in active BDM from a
    // prior session, the first try fails until BDMACT clears on the
    // fresh reset.
    for (uint8_t attempt = 0; attempt < 3; attempt++)
    {
        BKGD_LOW();
        BKGD_OUT();
        RST_LOW();
        RST_OUT();
        delayMicroseconds(2000);
        RST_IN();
        delay(50);
        bdm_enter_rst_after_release = PORT_READ(RST_BIT);
        if (bdm_enter_rst_after_release == 0)
        {
            BKGD_IN();
            if (attempt == 2)
                return 1;
            delay(50);
            continue;
        }
        BKGD_IN();
        delayMicroseconds(200);
        bdm_enter_bkgd_after_release = BKGD_READ();
        if (bdm_enter_bkgd_after_release == 0)
        {
            if (attempt == 2)
                return 2;
            delay(50);
            continue;
        }
        if (!bdm_sync())
        {
            if (attempt == 2)
                return 3;
            delay(50);
            continue;
        }
        if (!bdm_enable_ack())
        {
            if (attempt == 2)
                return 3;
            delay(50);
            continue;
        }
        // Freeze the chip's COP and RTI counters while in active BDM.
        // Without this, COP fires and resets the chip during long bulk
        // operations because the halted CPU cannot tickle ARMCOP.
        // COPCTL bit 6 (RSBCK)=1 freezes the counters; bit 5 (WRTMASK)=1
        // protects CR[2:0] and WCOP from being clobbered by this write.
        bdm_write_byte(COPCTL_REG, 0x60);
        return 0;
    }
    return 3;
}

// Flash module helpers
// FCLKDIV target = 1 MHz FCLK (RM 26.3.2.1 Table 26-9).
static bool fcmd_init_clkdiv()
{
    uint32_t bus_hz = 1000000000u / bdm.bus_period_ns;
    uint32_t fdiv = (bus_hz / 1000000u);
    if (fdiv > 0)
        fdiv -= 1;
    if (fdiv < 4)
        fdiv = 4;
    if (fdiv > 0x3F)
        fdiv = 0x3F;
    bdm_write_byte(FCLKDIV_REG, fdiv & 0x3F);
    return (bdm_read_byte(FCLKDIV_REG) & 0x80) != 0; // FDIVLD set on success
}

static void fcmd_clear_errors()
{
    bdm_write_byte(FSTAT_REG, FSTAT_ACCERR | FSTAT_FPVIOL);
}

static bool fcmd_wait_ccif(uint32_t timeout_ms = 5000)
{
    uint32_t t0 = millis();
    while (millis() - t0 < timeout_ms)
    {
        uint8_t fstat = bdm_read_byte(FSTAT_REG);
        if (bdm_last_err)
            continue;
        if (fstat & FSTAT_CCIF)
            return true;
    }
    return false;
}

// FCCOB load via indexed layout: (ix, hi, lo).
static inline void fccob(uint8_t ix, uint8_t hi, uint8_t lo)
{
    bdm_write_byte(FCCOBIX_REG, ix);
    bdm_write_byte(FCCOBHI_REG, hi);
    bdm_write_byte(FCCOBLO_REG, lo);
}

static bool fcmd_launch_and_check()
{
    bdm_write_byte(FSTAT_REG, FSTAT_CCIF); // launch
    if (!fcmd_wait_ccif())
        return false;
    uint8_t st = bdm_read_byte(FSTAT_REG);
    return (st & (FSTAT_ACCERR | FSTAT_FPVIOL)) == 0;
}

// Erase one 1 KB P-Flash sector.
static bool fcmd_erase_p_sector(uint32_t global_addr)
{
    if (!fcmd_wait_ccif())
        return false;
    fcmd_clear_errors();
    fccob(0, FCMD_ERASE_P_FLASH_SECTOR, (uint8_t)(global_addr >> 16));
    fccob(1, (uint8_t)(global_addr >> 8), (uint8_t)(global_addr));
    return fcmd_launch_and_check();
}

// Program one 8-byte phrase to P-Flash. global_addr must be 8-byte aligned.
static bool fcmd_program_p_phrase(uint32_t global_addr, const uint8_t *d)
{
    if (!fcmd_wait_ccif())
        return false;
    fcmd_clear_errors();
    fccob(0, FCMD_PROGRAM_P_FLASH, (uint8_t)(global_addr >> 16));
    fccob(1, (uint8_t)(global_addr >> 8), (uint8_t)(global_addr));
    fccob(2, d[0], d[1]);
    fccob(3, d[2], d[3]);
    fccob(4, d[4], d[5]);
    fccob(5, d[6], d[7]);
    return fcmd_launch_and_check();
}

// Erase one 256-byte D-Flash sector.
static bool fcmd_erase_d_sector(uint32_t global_addr)
{
    if (!fcmd_wait_ccif())
        return false;
    fcmd_clear_errors();
    fccob(0, FCMD_ERASE_D_SECTOR, (uint8_t)(global_addr >> 16));
    fccob(1, (uint8_t)(global_addr >> 8), (uint8_t)(global_addr));
    return fcmd_launch_and_check();
}

// Program one 4-byte phrase to D-Flash. global_addr must be 4-byte aligned.
// Final FCCOBIX at launch = 3 (signals 2 words = 4 bytes payload).
static bool fcmd_program_d_phrase(uint32_t global_addr, const uint8_t *d)
{
    if (!fcmd_wait_ccif())
        return false;
    fcmd_clear_errors();
    fccob(0, FCMD_PROGRAM_D_FLASH, (uint8_t)(global_addr >> 16));
    fccob(1, (uint8_t)(global_addr >> 8), (uint8_t)(global_addr));
    fccob(2, d[0], d[1]);
    fccob(3, d[2], d[3]);
    return fcmd_launch_and_check();
}

// Hex parsing / printing.
static uint32_t parse_hex(const char *&s)
{
    uint32_t v = 0;
    while (*s == ' ')
        s++;
    while ((*s >= '0' && *s <= '9') || (*s >= 'a' && *s <= 'f') || (*s >= 'A' && *s <= 'F'))
    {
        v <<= 4;
        if (*s <= '9')
            v |= *s - '0';
        else if (*s <= 'F')
            v |= *s - 'A' + 10;
        else
            v |= *s - 'a' + 10;
        s++;
    }
    return v;
}

static const char HEX_CHARS[] = "0123456789ABCDEF";
static inline void print_hex_byte(Stream &out, uint8_t b)
{
    out.write(HEX_CHARS[b >> 4]);
    out.write(HEX_CHARS[b & 0xF]);
}

// Write a page-select register with readback confirmation. Silent BDM
// write failures would otherwise leave PPAGE/EPAGE pointing at a stale
// page and subsequent reads would return the wrong sector's data.
static void bdm_write_page_register(uint16_t reg, uint8_t value)
{
    for (uint8_t r = 0; r < 6; r++)
    {
        bdm_write_byte(reg, value);
        if (bdm_read_byte(reg) == value)
            return;
    }
}

// Bulk-read P-Flash via the PPAGE window (0x8000..0xBFFF) for arbitrary
// PPAGE, or the fixed window (0xC000..0xFFFF) for PPAGE 0xFF.
// cur_page sentinel 0x00 is never a valid PPAGE on this chip, so the
// first byte of every transfer triggers a PPAGE write.
static void bulk_read_pflash(Stream &out, uint32_t addr, uint32_t count)
{
    uint8_t buf[4096];
    uint8_t cur_page = 0x00;
    uint32_t fail_at = 0xFFFFFFFFul;
    for (uint32_t i = 0; i < count; i++)
    {
        uint32_t ga = addr + i;
        uint8_t page = (uint8_t)(ga >> 14);
        uint16_t local = (uint16_t)(ga & 0x3FFF);
        uint16_t cpu_base = (page == 0xFF) ? 0xC000 : 0x8000;
        if (page != 0xFF && page != cur_page)
        {
            bdm_write_byte(PPAGE_REG, page);
            cur_page = page;
            if (bdm_last_err)
            {
                fail_at = i;
                break;
            }
        }
        buf[i] = bdm_read_byte((uint16_t)(cpu_base | local));
        if (bdm_last_err)
        {
            fail_at = i;
            break;
        }
    }
    if (fail_at != 0xFFFFFFFFul)
    {
        out.print(F("\nERR rpf bdm failure @ offset 0x"));
        out.println(fail_at, HEX);
        return;
    }
    for (uint32_t i = 0; i < count; i++)
        print_hex_byte(out, buf[i]);
    out.println();
}

// Bulk-read D-Flash via EPAGE-mapped 0x0800–0x0BFF window (1 KB per page).
static void bulk_read_dflash(Stream &out, uint32_t addr, uint32_t count)
{
    uint8_t buf[4096];
    uint8_t cur_epage = 0xFF; // sentinel
    bool first = true;        // forces EPAGE write on first byte
    uint32_t fail_at = 0xFFFFFFFFul;
    for (uint32_t i = 0; i < count; i++)
    {
        uint32_t ga = addr + i;
        uint32_t off = ga - 0x100000UL;
        uint8_t page = (uint8_t)((off >> 10) & 0xFF);
        uint16_t local = 0x0800 | (uint16_t)(off & 0x3FF);
        if (first || page != cur_epage)
        {
            bdm_write_byte(EPAGE_REG, page);
            cur_epage = page;
            first = false;
            if (bdm_last_err)
            {
                fail_at = i;
                break;
            }
        }
        buf[i] = bdm_read_byte(local);
        if (bdm_last_err)
        {
            fail_at = i;
            break;
        }
    }
    if (fail_at != 0xFFFFFFFFul)
    {
        out.print(F("\nERR rdflash bdm failure @ offset 0x"));
        out.println(fail_at, HEX);
        return;
    }
    for (uint32_t i = 0; i < count; i++)
        print_hex_byte(out, buf[i]);
    out.println();
}

// Stream-receive helper: read N bytes from Serial with timeout
static bool serial_read_exact(uint8_t *buf, uint32_t n, uint32_t timeout_ms = 30000)
{
    uint32_t got = 0;
    uint32_t t0 = millis();
    while (got < n)
    {
        if (Serial.available())
        {
            buf[got++] = (uint8_t)Serial.read();
            t0 = millis();
        }
        else if (millis() - t0 > timeout_ms)
        {
            return false;
        }
    }
    return true;
}

// Streaming P-Flash programmer. The host sends 256-byte bursts; the
// firmware ACKs each burst with '.'
static void cmd_wpflash(Stream &out, uint32_t base, uint32_t total)
{
    if (total == 0 || (total & 7) || (base & 7))
    {
        out.println(F("ERR base and length must be 8-byte aligned"));
        return;
    }
    if (!fcmd_init_clkdiv())
    {
        out.println(F("ERR FCLKDIV init failed"));
        return;
    }
    bdm_write_byte(COPCTL_REG, 0x60); // RSBCK | WRTMASK
    while (Serial.available())
        Serial.read();
    out.print(F("OK ready "));
    out.print(total, DEC);
    out.println(F(" 256"));

    constexpr uint16_t BURST = 256;
    uint8_t buf[BURST];
    uint32_t off = 0;
    uint32_t phrases_done = 0;
    while (off < total)
    {
        uint32_t n = total - off;
        if (n > BURST)
            n = BURST;
        if (!serial_read_exact(buf, n))
        {
            out.print(F("\nERR serial timeout @ 0x"));
            out.println(base + off, HEX);
            return;
        }
        for (uint32_t p = 0; p < n; p += 8)
        {
            uint32_t addr = base + off + p;
            bool ok = false;
            for (uint8_t a = 0; a < 3 && !ok; a++)
            {
                if (a > 0)
                    fcmd_init_clkdiv();
                ok = fcmd_program_p_phrase(addr, &buf[p]);
                if (ok)
                {
                    // Per-phrase verify via the read path used by rpf.
                    uint8_t page = (uint8_t)(addr >> 14);
                    uint16_t local = (uint16_t)(addr & 0x3FFF);
                    uint16_t cpu_base = (page == 0xFF) ? 0xC000 : 0x8000;
                    if (page != 0xFF)
                        bdm_write_page_register(PPAGE_REG, page);
                    for (uint8_t v = 0; v < 8; v++)
                    {
                        if (bdm_read_byte(cpu_base | (local + v)) != buf[p + v])
                        {
                            ok = false;
                            break;
                        }
                    }
                }
            }
            if (!ok)
            {
                uint8_t st = bdm_read_byte(FSTAT_REG);
                out.print(F("\nERR program failed @ 0x"));
                out.print(addr, HEX);
                out.print(F(" FSTAT=0x"));
                out.println(st, HEX);
                return;
            }
            phrases_done++;
        }
        off += n;
        out.write('.');
    }
    out.println();
    out.print(F("OK wpflash done: "));
    out.print(phrases_done);
    out.println(F(" phrases"));
}

// Streaming D-Flash programmer. Same shape as wpflash but 4-byte phrases.
static void cmd_wdflash(Stream &out, uint32_t base, uint32_t total)
{
    if (total == 0 || (total & 3) || (base & 3))
    {
        out.println(F("ERR base and length must be 4-byte aligned"));
        return;
    }
    if (!fcmd_init_clkdiv())
    {
        out.println(F("ERR FCLKDIV init failed"));
        return;
    }
    bdm_write_byte(COPCTL_REG, 0x60);
    bdm_write_byte(DFPROT_REG, 0x80); // DPOPEN, unprotect all D-Flash sectors
    while (Serial.available())
        Serial.read();
    out.print(F("OK ready "));
    out.print(total, DEC);
    out.println(F(" 256"));

    constexpr uint16_t BURST = 256;
    uint8_t buf[BURST];
    uint32_t off = 0;
    uint32_t phrases_done = 0;
    while (off < total)
    {
        uint32_t n = total - off;
        if (n > BURST)
            n = BURST;
        if (!serial_read_exact(buf, n))
        {
            out.print(F("\nERR serial timeout @ 0x"));
            out.println(base + off, HEX);
            return;
        }
        for (uint32_t p = 0; p < n; p += 4)
        {
            uint32_t addr = base + off + p;
            bool ok = false;
            for (uint8_t a = 0; a < 3 && !ok; a++)
            {
                if (a > 0)
                {
                    fcmd_init_clkdiv();
                    bdm_write_byte(DFPROT_REG, 0x80);
                }
                ok = fcmd_program_d_phrase(addr, &buf[p]);
                if (ok)
                {
                    uint32_t df_off = addr - 0x100000UL;
                    uint8_t page = (uint8_t)((df_off >> 10) & 0xFF);
                    uint16_t local = 0x0800 | (uint16_t)(df_off & 0x3FF);
                    bdm_write_page_register(EPAGE_REG, page);
                    for (uint8_t v = 0; v < 4; v++)
                    {
                        if (bdm_read_byte(local + v) != buf[p + v])
                        {
                            ok = false;
                            break;
                        }
                    }
                }
            }
            if (!ok)
            {
                uint8_t st = bdm_read_byte(FSTAT_REG);
                out.print(F("\nERR program failed @ 0x"));
                out.print(addr, HEX);
                out.print(F(" FSTAT=0x"));
                out.println(st, HEX);
                return;
            }
            phrases_done++;
        }
        off += n;
        out.write('.');
    }
    out.println();
    out.print(F("OK wdflash done: "));
    out.print(phrases_done);
    out.println(F(" phrases"));
}

// Command dispatch
static void print_help(Stream &out)
{
    out.println(F("frm3_bdm_v2 commands:"));
    out.println(F("  help                          : this list"));
    out.println(F("  probe                         : passive BKGD / RESET line state"));
    out.println(F("  sync                          : measure bus period"));
    out.println(F("  enter                         : drive RESET + latch active BDM"));
    out.println(F("  status                        : FSEC / FCLKDIV / FSTAT etc."));
    out.println(F("  rb <addr>                     : read byte"));
    out.println(F("  wb <addr> <val>               : write byte"));
    out.println(F("  rw <addr>                     : read word"));
    out.println(F("  ww <addr> <val16>             : write word"));
    out.println(F("  rpf <gaddr> <count>           : bulk read P-Flash (hex, ≤4096)"));
    out.println(F("  rdflash <gaddr> <count>       : bulk read D-Flash (hex, ≤4096)"));
    out.println(F("  epsec <gaddr>                 : erase one 1 KB P-Flash sector"));
    out.println(F("  edsec <gaddr>                 : erase one 256 B D-Flash sector"));
    out.println(F("  edflash                       : erase all 128 D-Flash sectors"));
    out.println(F("  wpflash <gaddr> <len>         : stream P-Flash program"));
    out.println(F("  wdflash <gaddr> <len>         : stream D-Flash program"));
    out.println(F("OK"));
}

static void cmd_status(Stream &out)
{
    if (!bdm.synced)
    {
        out.println(F("ERR not synced"));
        return;
    }
    uint8_t fsec = bdm_read_byte(FSEC_REG);
    uint8_t fclk = bdm_read_byte(FCLKDIV_REG);
    uint8_t fstat = bdm_read_byte(FSTAT_REG);
    uint8_t ferst = bdm_read_byte(FERSTAT_REG);
    uint8_t fprot = bdm_read_byte(FPROT_REG);
    uint8_t dfprot = bdm_read_byte(DFPROT_REG);
    out.print(F("OK FSEC=0x"));
    out.print(fsec, HEX);
    out.print(F(" FCLKDIV=0x"));
    out.print(fclk, HEX);
    out.print(F(" FSTAT=0x"));
    out.print(fstat, HEX);
    out.print(F(" FERSTAT=0x"));
    out.print(ferst, HEX);
    out.print(F(" FPROT=0x"));
    out.print(fprot, HEX);
    out.print(F(" DFPROT=0x"));
    out.print(dfprot, HEX);
    out.print(F(" bus_period_ns="));
    out.print(bdm.bus_period_ns);
    out.print(F(" ack="));
    out.println(bdm.ack_enabled ? F("on") : F("off"));
}

static void handle_line(char *line, Stream &out)
{
    char *p = line;
    while (*p == ' ')
        p++;
    char *cmd = p;
    while (*p && *p != ' ')
        p++;
    if (*p)
        *p++ = 0;
    const char *args = p;

    if (cmd[0] == 0 || !strcmp(cmd, "help"))
    {
        print_help(out);
    }
    else if (!strcmp(cmd, "probe"))
    {
        BKGD_IN();
        pinMode(RESET_PIN, INPUT);
        delayMicroseconds(200);
        uint8_t bkgd = BKGD_READ();
        uint8_t rst = digitalRead(RESET_PIN);
        out.print(F("OK BKGD="));
        out.print(bkgd ? F("HIGH") : F("LOW"));
        out.print(F(" RESET="));
        out.println(rst ? F("HIGH") : F("LOW"));
    }
    else if (!strcmp(cmd, "sync"))
    {
        if (bdm_sync())
        {
            out.print(F("OK bus_period_ns="));
            out.println(bdm.bus_period_ns);
        }
        else
        {
            out.println(F("ERR sync failed"));
        }
    }
    else if (!strcmp(cmd, "enter"))
    {
        uint8_t r = bdm_enter();
        if (r == 0)
        {
            out.print(F("OK in active BDM, bus_period_ns="));
            out.print(bdm.bus_period_ns);
            out.print(F(" ack="));
            out.println(bdm.ack_enabled ? F("on") : F("off"));
        }
        else
        {
            out.print(F("ERR enter failed (code "));
            out.print(r);
            out.println(F(": 1=RESET stuck, 2=BKGD low, 3=SYNC/ACK failed)"));
        }
    }
    else if (!strcmp(cmd, "status"))
    {
        cmd_status(out);
    }
    else if (!strcmp(cmd, "rdb"))
    {
        if (!bdm.synced)
        {
            out.println(F("ERR not synced"));
            return;
        }
        uint16_t a = (uint16_t)parse_hex(args);
        out.print(F("OK rdb 0x"));
        out.print(a, HEX);
        out.print(F(" = 0x"));
        out.println(bdm_read_bd_byte(a), HEX);
    }
    else if (!strcmp(cmd, "wbdb"))
    {
        if (!bdm.synced)
        {
            out.println(F("ERR not synced"));
            return;
        }
        uint16_t a = (uint16_t)parse_hex(args);
        uint8_t v = (uint8_t)parse_hex(args);
        bdm_write_bd_byte(a, v);
        out.print(F("OK wbdb 0x"));
        out.print(a, HEX);
        out.print(F(" = 0x"));
        out.println(v, HEX);
    }
    else if (!strcmp(cmd, "rb"))
    {
        if (!bdm.synced)
        {
            out.println(F("ERR not synced"));
            return;
        }
        uint16_t a = (uint16_t)parse_hex(args);
        out.print(F("OK rb 0x"));
        out.print(a, HEX);
        out.print(F(" = 0x"));
        out.println(bdm_read_byte(a), HEX);
    }
    else if (!strcmp(cmd, "wb"))
    {
        if (!bdm.synced)
        {
            out.println(F("ERR not synced"));
            return;
        }
        uint16_t a = (uint16_t)parse_hex(args);
        uint8_t v = (uint8_t)parse_hex(args);
        bdm_write_byte(a, v);
        out.print(F("OK wb 0x"));
        out.print(a, HEX);
        out.print(F(" = 0x"));
        out.println(v, HEX);
    }
    else if (!strcmp(cmd, "rw"))
    {
        if (!bdm.synced)
        {
            out.println(F("ERR not synced"));
            return;
        }
        uint16_t a = (uint16_t)parse_hex(args);
        out.print(F("OK rw 0x"));
        out.print(a, HEX);
        out.print(F(" = 0x"));
        out.println(bdm_read_word(a), HEX);
    }
    else if (!strcmp(cmd, "ww"))
    {
        if (!bdm.synced)
        {
            out.println(F("ERR not synced"));
            return;
        }
        uint16_t a = (uint16_t)parse_hex(args);
        uint16_t v = (uint16_t)parse_hex(args);
        bdm_write_word(a, v);
        out.print(F("OK ww 0x"));
        out.print(a, HEX);
        out.print(F(" = 0x"));
        out.println(v, HEX);
    }
    else if (!strcmp(cmd, "rpf"))
    {
        if (!bdm.synced)
        {
            out.println(F("ERR not synced"));
            return;
        }
        uint32_t a = parse_hex(args);
        uint32_t n = parse_hex(args);
        if (n == 0 || n > 4096)
        {
            out.println(F("ERR count must be 1..4096"));
            return;
        }
        out.print(F("OK rpf 0x"));
        out.print(a, HEX);
        out.print(F(" "));
        out.println(n, DEC);
        bulk_read_pflash(out, a, n);
    }
    else if (!strcmp(cmd, "rdflash"))
    {
        if (!bdm.synced)
        {
            out.println(F("ERR not synced"));
            return;
        }
        uint32_t a = parse_hex(args);
        uint32_t n = parse_hex(args);
        if (n == 0 || n > 4096)
        {
            out.println(F("ERR count must be 1..4096"));
            return;
        }
        out.print(F("OK rdflash 0x"));
        out.print(a, HEX);
        out.print(F(" "));
        out.println(n, DEC);
        bulk_read_dflash(out, a, n);
    }
    else if (!strcmp(cmd, "epsec"))
    {
        if (!bdm.synced)
        {
            out.println(F("ERR not synced"));
            return;
        }
        uint32_t a = parse_hex(args);
        uint32_t sect = a & ~0x3FFu;
        if (sect >= FSEC_SECTOR_BASE)
        {
            out.print(F("ERR refuse to erase FSEC sector @ 0x"));
            out.println(sect, HEX);
            return;
        }
        if (!fcmd_init_clkdiv())
        {
            out.println(F("ERR FCLKDIV init failed"));
            return;
        }
        bdm_write_byte(COPCTL_REG, 0x60);
        if (fcmd_erase_p_sector(sect))
        {
            out.print(F("OK epsec 0x"));
            out.println(sect, HEX);
        }
        else
        {
            uint8_t st = bdm_read_byte(FSTAT_REG);
            out.print(F("ERR epsec failed @ 0x"));
            out.print(sect, HEX);
            out.print(F(" FSTAT=0x"));
            out.println(st, HEX);
        }
    }
    else if (!strcmp(cmd, "edsec"))
    {
        if (!bdm.synced)
        {
            out.println(F("ERR not synced"));
            return;
        }
        uint32_t a = parse_hex(args);
        uint32_t sect = a & ~0xFFu;
        if (!fcmd_init_clkdiv())
        {
            out.println(F("ERR FCLKDIV init failed"));
            return;
        }
        bdm_write_byte(COPCTL_REG, 0x60);
        bdm_write_byte(DFPROT_REG, 0x80);
        if (fcmd_erase_d_sector(sect))
        {
            out.print(F("OK edsec 0x"));
            out.println(sect, HEX);
        }
        else
        {
            uint8_t st = bdm_read_byte(FSTAT_REG);
            out.print(F("ERR edsec failed @ 0x"));
            out.print(sect, HEX);
            out.print(F(" FSTAT=0x"));
            out.println(st, HEX);
        }
    }
    else if (!strcmp(cmd, "edflash"))
    {
        if (!bdm.synced)
        {
            out.println(F("ERR not synced"));
            return;
        }
        if (!fcmd_init_clkdiv())
        {
            out.println(F("ERR FCLKDIV init failed"));
            return;
        }
        bdm_write_byte(COPCTL_REG, 0x60);
        bdm_write_byte(DFPROT_REG, 0x80);
        uint16_t erased = 0, failed = 0;
        for (uint32_t sec = 0; sec < 128; sec++)
        {
            uint32_t addr = 0x100000UL + sec * 256;
            if (fcmd_erase_d_sector(addr))
                erased++;
            else
                failed++;
            if ((sec & 0x0F) == 0x0F)
                out.write('.');
        }
        out.println();
        out.print(F("OK edflash erased "));
        out.print(erased);
        out.print(F(" failed "));
        out.println(failed);
    }
    else if (!strcmp(cmd, "fullpartition"))
    {
        // DESTRUCTIVE: erases all D-Flash and programs the EEE NV info reg
        // with the given DFPART/ERPART sector counts. For FRM3 use
        //   fullpartition 0 16
        // (0 user-D-Flash sectors, 16 EEE buffer RAM sectors = full 4 KB EEE).
        // Constraints from RM §26.4.2.15:
        //   DFPART ≤ 128, ERPART ≤ 16
        //   if ERPART>0: 128-DFPART ≥ 12, (128-DFPART)/ERPART ≥ 8
        if (!bdm.synced)
        {
            out.println(F("ERR not synced"));
            return;
        }
        const char *a = args;
        uint32_t dfpart = 0;
        while (*a >= '0' && *a <= '9')
        {
            dfpart = dfpart * 10 + (*a - '0');
            a++;
        }
        while (*a == ' ')
            a++;
        uint32_t erpart = 0;
        while (*a >= '0' && *a <= '9')
        {
            erpart = erpart * 10 + (*a - '0');
            a++;
        }
        if (dfpart > 128 || erpart > 16)
        {
            out.println(F("ERR DFPART must be 0..128, ERPART must be 0..16"));
            return;
        }
        if (erpart > 0 && (128u - dfpart) < 12u)
        {
            out.println(F("ERR (128-DFPART) must be >=12 when ERPART>0"));
            return;
        }
        if (erpart > 0 && ((128u - dfpart) / erpart) < 8u)
        {
            out.println(F("ERR (128-DFPART)/ERPART must be >=8"));
            return;
        }
        if (!fcmd_init_clkdiv())
        {
            out.println(F("ERR FCLKDIV init failed"));
            return;
        }
        bdm_write_byte(COPCTL_REG, 0x60);
        if (!fcmd_wait_ccif())
        {
            out.println(F("ERR CCIF stuck"));
            return;
        }
        fcmd_clear_errors();
        fccob(0, FCMD_FULL_PARTITION_D, 0);
        fccob(1, (uint8_t)(dfpart >> 8), (uint8_t)(dfpart & 0xFF));
        fccob(2, (uint8_t)(erpart >> 8), (uint8_t)(erpart & 0xFF));
        // FULL_PARTITION_D erases all D-Flash plus the EEE NV info reg -
        // takes a long time (up to several seconds). Use a generous timeout.
        bdm_write_byte(FSTAT_REG, FSTAT_CCIF); // launch
        if (!fcmd_wait_ccif(20000))
        {
            out.println(F("ERR fullpartition CCIF timeout"));
            return;
        }
        uint8_t st = bdm_read_byte(FSTAT_REG);
        if (st & (FSTAT_ACCERR | FSTAT_FPVIOL))
        {
            out.print(F("ERR fullpartition FSTAT=0x"));
            out.println(st, HEX);
            return;
        }
        out.print(F("OK fullpartition DFPART="));
        out.print(dfpart);
        out.print(F(" ERPART="));
        out.println(erpart);
    }
    else if (!strcmp(cmd, "enableeee"))
    {
        // Start the EEE engine. After this, writes to global 0x13_F000-
        // 0x13_FFFF are automatically logged to D-Flash. Requires a prior
        // fullpartition (otherwise ACCERR per §26.4.2.19).
        if (!bdm.synced)
        {
            out.println(F("ERR not synced"));
            return;
        }
        if (!fcmd_init_clkdiv())
        {
            out.println(F("ERR FCLKDIV init failed"));
            return;
        }
        if (!fcmd_wait_ccif())
        {
            out.println(F("ERR CCIF stuck"));
            return;
        }
        fcmd_clear_errors();
        fccob(0, FCMD_ENABLE_EEE, 0);
        if (fcmd_launch_and_check())
        {
            out.println(F("OK EEE enabled"));
        }
        else
        {
            uint8_t st = bdm_read_byte(FSTAT_REG);
            out.print(F("ERR enableeee FSTAT=0x"));
            out.println(st, HEX);
        }
    }
    else if (!strcmp(cmd, "disableeee"))
    {
        if (!bdm.synced)
        {
            out.println(F("ERR not synced"));
            return;
        }
        if (!fcmd_init_clkdiv())
        {
            out.println(F("ERR FCLKDIV init failed"));
            return;
        }
        if (!fcmd_wait_ccif())
        {
            out.println(F("ERR CCIF stuck"));
            return;
        }
        fcmd_clear_errors();
        fccob(0, FCMD_DISABLE_EEE, 0);
        if (fcmd_launch_and_check())
            out.println(F("OK EEE disabled"));
        else
        {
            uint8_t st = bdm_read_byte(FSTAT_REG);
            out.print(F("ERR disableeee FSTAT=0x"));
            out.println(st, HEX);
        }
    }
    else if (!strcmp(cmd, "ev_section"))
    {
        // ev_section <global_addr_hex> <num_phrases_dec>
        // Runs ERASE_VERIFY_P_FLASH_SECTION (FCMD 0x03) non-destructively.
        // If the address is invalid (page doesn't physically exist on this
        // chip), the FTM returns ACCERR. Used to probe chip flash size.
        if (!bdm.synced)
        {
            out.println(F("ERR not synced"));
            return;
        }
        const char *a = args;
        uint32_t addr = parse_hex(a);
        while (*a == ' ')
            a++;
        uint32_t nphr = strtoul(a, nullptr, 10);
        if (nphr == 0)
            nphr = 1;
        if (!fcmd_init_clkdiv())
        {
            out.println(F("ERR FCLKDIV init failed"));
            return;
        }
        if (!fcmd_wait_ccif())
        {
            out.println(F("ERR CCIF stuck"));
            return;
        }
        fcmd_clear_errors();
        fccob(0, 0x03 /* ERASE_VERIFY_P_FLASH_SECTION */, (uint8_t)(addr >> 16));
        fccob(1, (uint8_t)(addr >> 8), (uint8_t)addr);
        fccob(2, (uint8_t)(nphr >> 8), (uint8_t)nphr);
        bdm_write_byte(FSTAT_REG, FSTAT_CCIF);
        if (!fcmd_wait_ccif())
        {
            out.println(F("ERR CCIF timeout"));
            return;
        }
        uint8_t st = bdm_read_byte(FSTAT_REG);
        out.print(F("OK ev_section addr=0x"));
        out.print(addr, HEX);
        out.print(F(" phrases="));
        out.print(nphr);
        out.print(F(" FSTAT=0x"));
        out.print(st, HEX);
        if (st & FSTAT_ACCERR)
            out.print(F(" [ACCERR - address invalid]"));
        if (st & FSTAT_FPVIOL)
            out.print(F(" [FPVIOL]"));
        if (st & 0x40)
            out.print(F(" [MGSTAT1 - flagged]"));
        if (st & 0x20)
            out.print(F(" [MGSTAT0 - flagged]"));
        if ((st & (FSTAT_ACCERR | FSTAT_FPVIOL)) == 0)
        {
            if ((st & 0xC0) == 0xC0)
                out.print(F(" - DOUBLE FAULT"));
            else if (st & 0x40)
                out.print(F(" - NOT BLANK"));
            else if (st & 0x20)
                out.print(F(" - single bit error"));
            else
                out.print(F(" - BLANK (page exists, all 0xFF)"));
        }
        out.println();
    }
    else if (!strcmp(cmd, "eeequery"))
    {
        // Report current EEE partition + status. Pre-partition values are
        // DFPART=0xFFFF / ERPART=0xFFFF (per §26.4.2.21).
        if (!bdm.synced)
        {
            out.println(F("ERR not synced"));
            return;
        }
        if (!fcmd_init_clkdiv())
        {
            out.println(F("ERR FCLKDIV init failed"));
            return;
        }
        if (!fcmd_wait_ccif())
        {
            out.println(F("ERR CCIF stuck"));
            return;
        }
        fcmd_clear_errors();
        fccob(0, FCMD_EEE_QUERY, 0);
        bdm_write_byte(FSTAT_REG, FSTAT_CCIF);
        if (!fcmd_wait_ccif())
        {
            out.println(F("ERR eeequery CCIF timeout"));
            return;
        }
        uint8_t st = bdm_read_byte(FSTAT_REG);
        if (st & (FSTAT_ACCERR | FSTAT_FPVIOL))
        {
            out.print(F("ERR eeequery FSTAT=0x"));
            out.println(st, HEX);
            return;
        }
        auto get = [&](uint8_t ix) -> uint16_t
        {
            bdm_write_byte(FCCOBIX_REG, ix);
            return ((uint16_t)bdm_read_byte(FCCOBHI_REG) << 8) | bdm_read_byte(FCCOBLO_REG);
        };
        uint16_t dfpart = get(1);
        uint16_t erpart = get(2);
        uint16_t ecount = get(3);
        uint16_t dr = get(4);
        out.print(F("OK eeequery DFPART="));
        out.print(dfpart);
        out.print(F(" ERPART="));
        out.print(erpart);
        out.print(F(" ECOUNT="));
        out.print(ecount);
        out.print(F(" DEAD="));
        out.print(dr >> 8);
        out.print(F(" READY="));
        out.println(dr & 0xFF);
    }
    else if (!strcmp(cmd, "reee"))
    {
        // Bulk-read EEE buffer RAM (4 KB at global 0x13_F000-0x13_FFFF).
        // Uses BDMGPR (BGAE=1, BGP6..0=0x13) for direct global addressing.
        // Args: <count_hex>  OR  <base> <count>  (same dual form as weee).
        if (!bdm.synced)
        {
            out.println(F("ERR not synced"));
            return;
        }
        const char *a = args;
        uint32_t first = parse_hex(a);
        while (*a == ' ')
            a++;
        uint32_t second = parse_hex(a);
        uint32_t count = (second != 0) ? second : first;
        if (count == 0 || count > EEE_BUF_SIZE)
        {
            out.println(F("ERR count must be 1..4096"));
            return;
        }
        bdm_write_bd_byte(BDMGPR_BD_ADDR, 0x80 | EEE_BUF_GLOBAL_HI);
        out.print(F("OK reee "));
        out.println(count, DEC);
        uint8_t buf[4096];
        uint32_t fail_at = 0xFFFFFFFFul;
        for (uint32_t i = 0; i < count; i++)
        {
            buf[i] = bdm_read_byte(EEE_BUF_LOCAL_BASE + i);
            if (bdm_last_err)
            {
                fail_at = i;
                break;
            }
        }
        bdm_write_bd_byte(BDMGPR_BD_ADDR, 0);
        if (fail_at != 0xFFFFFFFFul)
        {
            out.print(F("\nERR reee bdm failure @ 0x"));
            out.println(fail_at, HEX);
            return;
        }
        for (uint32_t i = 0; i < count; i++)
            print_hex_byte(out, buf[i]);
        out.println();
    }
    else if (!strcmp(cmd, "weee"))
    {
        // Stream-write EEE buffer RAM. Requires fullpartition + enableeee
        // to have been run first so the EEE engine logs each write to
        // D-Flash for persistence. Args: <total_hex>  OR  <base> <total>
        // (the base form is accepted so the host's generic stream-program
        // helper, which always emits "<cmd> <base> <total>", just works).
        if (!bdm.synced)
        {
            out.println(F("ERR not synced"));
            return;
        }
        const char *a = args;
        uint32_t first = parse_hex(a);
        while (*a == ' ')
            a++;
        uint32_t second = parse_hex(a);
        uint32_t total = (second != 0) ? second : first;
        if (total == 0 || total > EEE_BUF_SIZE)
        {
            out.println(F("ERR total must be 1..4096"));
            return;
        }
        bdm_write_bd_byte(BDMGPR_BD_ADDR, 0x80 | EEE_BUF_GLOBAL_HI);
        while (Serial.available())
            Serial.read();
        out.print(F("OK ready "));
        out.print(total, DEC);
        out.println(F(" 256"));
        constexpr uint16_t BURST = 256;
        uint8_t buf[BURST];
        uint32_t off = 0;
        while (off < total)
        {
            uint32_t n = total - off;
            if (n > BURST)
                n = BURST;
            if (!serial_read_exact(buf, n))
            {
                bdm_write_bd_byte(BDMGPR_BD_ADDR, 0);
                out.print(F("\nERR weee serial timeout @ 0x"));
                out.println(off, HEX);
                return;
            }
            for (uint32_t i = 0; i < n; i++)
            {
                bdm_write_byte((uint16_t)(EEE_BUF_LOCAL_BASE + off + i), buf[i]);
            }
            off += n;
            out.write('.');
        }
        bdm_write_bd_byte(BDMGPR_BD_ADDR, 0);
        out.println();
        out.print(F("OK weee done: "));
        out.print(off);
        out.println(F(" bytes"));
    }
    else if (!strcmp(cmd, "wpflash"))
    {
        if (!bdm.synced)
        {
            out.println(F("ERR not synced"));
            return;
        }
        const char *a = args;
        uint32_t base = parse_hex(a);
        uint32_t total = parse_hex(a);
        cmd_wpflash(out, base, total);
    }
    else if (!strcmp(cmd, "wdflash"))
    {
        if (!bdm.synced)
        {
            out.println(F("ERR not synced"));
            return;
        }
        const char *a = args;
        uint32_t base = parse_hex(a);
        uint32_t total = parse_hex(a);
        cmd_wdflash(out, base, total);

        // ---- CPU debug -------------------------------------------------------
    }
    else if (!strcmp(cmd, "letrun"))
    {
        // letrun [ms]
        //   Enter active BDM at reset (CPU halted at reset vector), GO to
        //   release the CPU into firmware, wait [ms] (default 500), then
        //   BACKGROUND to halt back. Avoids sync-after-normal-reset failure (the firmware
        //   may enter STOP/WAIT which kills the BDM clock).
        uint32_t ms = 500;
        const char *a = args;
        while (*a == ' ')
            a++;
        if (*a)
            ms = strtoul(a, nullptr, 10);
        uint8_t r = bdm_enter();
        if (r)
        {
            out.print(F("ERR bdm_enter code "));
            out.println(r);
            return;
        }
        uint8_t sts0 = bdm_read_bd_byte(BDMSTS_BD_ADDR);
        if (!(sts0 & 0x40))
        {
            out.print(F("ERR not in active BDM after enter, BDMSTS=0x"));
            out.println(sts0, HEX);
            return;
        }
        // Active-BDM-at-reset halts the CPU BEFORE the reset vector is
        // fetched, so PC sits in BDM ROM and GO would resume there. Read
        // the reset vector from 0xFFFE:FFFF and install it via WRITE_PC
        // so GO actually starts user firmware.
        uint8_t rvh = bdm_read_byte(0xFFFE);
        uint8_t rvl = bdm_read_byte(0xFFFF);
        uint16_t reset_vec = ((uint16_t)rvh << 8) | rvl;
        bdm_write_cpu_reg(BDM_WRITE_PC, reset_vec);
        // GO: leaves active BDM, CPU runs from PC (now reset vector).
        __disable_irq();
        bdm_send_byte(BDM_GO);
        __enable_irq();
        delay(ms);
        r = bdm_halt();
        if (r)
        {
            out.print(F("ERR halt code "));
            out.println(r);
            return;
        }
        // After halt the CPU is in active BDM. Read PC + key regs and
        // dump them in one response so the host doesn't waste time on
        // round-trips while the WDT is ticking.
        uint16_t pc = bdm_read_cpu_reg(BDM_READ_PC);
        uint16_t d = bdm_read_cpu_reg(BDM_READ_D);
        uint16_t x = bdm_read_cpu_reg(BDM_READ_X);
        uint16_t y = bdm_read_cpu_reg(BDM_READ_Y);
        uint16_t sp = bdm_read_cpu_reg(BDM_READ_SP);
        uint8_t ppage = bdm_read_byte(PPAGE_REG);
        out.print(F("OK halted after "));
        out.print(ms);
        out.print(F(" ms"));
        out.print(F(" PC=0x"));
        out.print(pc, HEX);
        out.print(F(" PPAGE=0x"));
        out.print(ppage, HEX);
        out.print(F(" D=0x"));
        out.print(d, HEX);
        out.print(F(" X=0x"));
        out.print(x, HEX);
        out.print(F(" Y=0x"));
        out.print(y, HEX);
        out.print(F(" SP=0x"));
        out.println(sp, HEX);
    }
    else if (!strcmp(cmd, "letrun_us"))
    {
        // letrun_us <us>
        //   Same as letrun, but with microsecond granularity for tight
        //   bisection of crash points. delayMicroseconds() is bounded
        //   (~16383 us safe), so cap at 16000 us.
        uint32_t us = 1000;
        const char *a = args;
        while (*a == ' ')
            a++;
        if (*a)
            us = strtoul(a, nullptr, 10);
        if (us > 16000)
            us = 16000;
        uint8_t r = bdm_enter();
        if (r)
        {
            out.print(F("ERR bdm_enter code "));
            out.println(r);
            return;
        }
        uint8_t rvh = bdm_read_byte(0xFFFE);
        uint8_t rvl = bdm_read_byte(0xFFFF);
        uint16_t reset_vec = ((uint16_t)rvh << 8) | rvl;
        bdm_write_cpu_reg(BDM_WRITE_PC, reset_vec);
        __disable_irq();
        bdm_send_byte(BDM_GO);
        __enable_irq();
        delayMicroseconds(us);
        r = bdm_halt();
        if (r)
        {
            out.print(F("ERR halt code "));
            out.println(r);
            return;
        }
        uint16_t pc = bdm_read_cpu_reg(BDM_READ_PC);
        uint16_t d = bdm_read_cpu_reg(BDM_READ_D);
        uint16_t x = bdm_read_cpu_reg(BDM_READ_X);
        uint16_t y = bdm_read_cpu_reg(BDM_READ_Y);
        uint16_t sp = bdm_read_cpu_reg(BDM_READ_SP);
        uint8_t ppage = bdm_read_byte(PPAGE_REG);
        out.print(F("OK halted after "));
        out.print(us);
        out.print(F(" us"));
        out.print(F(" PC=0x"));
        out.print(pc, HEX);
        out.print(F(" PPAGE=0x"));
        out.print(ppage, HEX);
        out.print(F(" D=0x"));
        out.print(d, HEX);
        out.print(F(" X=0x"));
        out.print(x, HEX);
        out.print(F(" Y=0x"));
        out.print(y, HEX);
        out.print(F(" SP=0x"));
        out.println(sp, HEX);
    }
    else if (!strcmp(cmd, "rcpu"))
    {
        // Read CPU registers. Requires CPU already halted in active BDM
        // (run letrun, or halt manually).
        if (!bdm.synced)
        {
            out.println(F("ERR not synced"));
            return;
        }
        uint16_t pc = bdm_read_cpu_reg(BDM_READ_PC);
        uint16_t d = bdm_read_cpu_reg(BDM_READ_D);
        uint16_t x = bdm_read_cpu_reg(BDM_READ_X);
        uint16_t y = bdm_read_cpu_reg(BDM_READ_Y);
        uint16_t sp = bdm_read_cpu_reg(BDM_READ_SP);
        uint8_t ppage = bdm_read_byte(PPAGE_REG);
        uint8_t sts = bdm_read_bd_byte(BDMSTS_BD_ADDR);
        out.print(F("OK PC=0x"));
        out.print(pc, HEX);
        out.print(F(" PPAGE=0x"));
        out.print(ppage, HEX);
        out.print(F(" D=0x"));
        out.print(d, HEX);
        out.print(F(" X=0x"));
        out.print(x, HEX);
        out.print(F(" Y=0x"));
        out.print(y, HEX);
        out.print(F(" SP=0x"));
        out.print(sp, HEX);
        out.print(F(" BDMSTS=0x"));
        out.println(sts, HEX);
    }
    else if (!strcmp(cmd, "wpc"))
    {
        // Write user PC. Roundtrip test for BDM firmware command path.
        if (!bdm.synced)
        {
            out.println(F("ERR not synced"));
            return;
        }
        const char *a = args;
        while (*a == ' ')
            a++;
        uint32_t v = strtoul(a, nullptr, 16);
        bdm_write_cpu_reg(BDM_WRITE_PC, (uint16_t)v);
        out.print(F("OK wpc 0x"));
        out.println(v, HEX);
    }
    else if (!strcmp(cmd, "halt"))
    {
        // Manual halt (CPU must be running in normal mode).
        if (!bdm.synced)
        {
            out.println(F("ERR not synced"));
            return;
        }
        uint8_t r = bdm_halt();
        if (r)
        {
            out.print(F("ERR halt code "));
            out.println(r);
            return;
        }
        uint16_t pc = bdm_read_cpu_reg(BDM_READ_PC);
        uint8_t ppage = bdm_read_byte(PPAGE_REG);
        out.print(F("OK halted PC=0x"));
        out.print(pc, HEX);
        out.print(F(" PPAGE=0x"));
        out.println(ppage, HEX);
    }
    else if (!strcmp(cmd, "go"))
    {
        // Resume CPU. Leaves active BDM - subsequent firmware commands fail
        // until the next halt.
        __disable_irq();
        bdm_send_byte(BDM_GO);
        __enable_irq();
        out.println(F("OK go"));
    }
    else if (!strcmp(cmd, "step"))
    {
        // Single-instruction step. CPU stays halted afterwards.
        if (!bdm.synced)
        {
            out.println(F("ERR not synced"));
            return;
        }
        __disable_irq();
        bdm_last_err = false;
        bdm_send_byte(BDM_TRACE1);
        bool ok = bdm_post_ack();
        __enable_irq();
        if (!ok)
        {
            out.println(F("ERR step ack"));
            return;
        }
        uint16_t pc = bdm_read_cpu_reg(BDM_READ_PC);
        uint8_t ppage = bdm_read_byte(PPAGE_REG);
        out.print(F("OK step PC=0x"));
        out.print(pc, HEX);
        out.print(F(" PPAGE=0x"));
        out.println(ppage, HEX);
    }
    else
    {
        out.print(F("ERR unknown command: "));
        out.println(cmd);
    }
}

// Arduino entry points
void setup()
{
    Serial.begin(1000000);
    dwt_init();
    BKGD_IN();
    RST_IN();
    pinMode(RESET_PIN, INPUT);
    delay(100);
    Serial.println();
    Serial.println(F("frm3_bdm_v2 ready. 'help' for commands."));
}

void loop()
{
    static char buf[64];
    static uint8_t n = 0;
    while (Serial.available())
    {
        int c = Serial.read();
        if (c == '\r')
            continue;
        if (c == '\n')
        {
            buf[n] = 0;
            if (n > 0)
                handle_line(buf, Serial);
            n = 0;
        }
        else if (n < sizeof(buf) - 1)
        {
            buf[n++] = (char)c;
        }
    }
}
