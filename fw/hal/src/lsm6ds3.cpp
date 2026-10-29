#include "hal/lsm6ds3.hpp"

#include <array>
#include <span>

namespace {

// Same addresses and the bits used here on LSM6DS3 (DocID026899) and LSM6DS3TR-C.
constexpr std::uint8_t reg_int1_ctrl = 0x0D;
constexpr std::uint8_t reg_who_am_i = 0x0F;
constexpr std::uint8_t reg_ctrl1_xl = 0x10;
constexpr std::uint8_t reg_ctrl3_c = 0x12;
constexpr std::uint8_t reg_status = 0x1E;
constexpr std::uint8_t reg_outx_l_g = 0x22;

constexpr std::uint8_t odr_104hz = 0b0100 << 4U; // ODR_XL / ODR_G, Tables 47 and 51
constexpr std::uint8_t fs_xl_2g = 0b00 << 2U;
constexpr std::uint8_t fs_g_250dps = 0b00 << 2U;
constexpr std::uint8_t ctrl3_bdu = 1U << 6U;
constexpr std::uint8_t ctrl3_if_inc = 1U << 2U;
// With equal ODRs DRDY_G comes after DRDY_XL, so it means both are ready (AN4650 3.1).
constexpr std::uint8_t int1_drdy_g = 1U << 1U;
constexpr std::uint8_t status_xlda = 1U << 0U;
constexpr std::uint8_t status_gda = 1U << 1U;

static_assert(hal::lsm6ds3_ctrl1_xl == (odr_104hz | fs_xl_2g));
static_assert(hal::lsm6ds3_ctrl2_g == (odr_104hz | fs_g_250dps));

constexpr std::array addresses{hal::lsm6ds3_addr_sa0_high, hal::lsm6ds3_addr_sa0_low};

} // namespace

namespace hal {

std::expected<Lsm6ds3Probe, I2cError> lsm6ds3_probe() {
    I2cError error = I2cError::nack;
    for (const std::uint8_t addr : addresses) {
        const auto who_am_i = i2c_read_reg(addr, reg_who_am_i);
        if (who_am_i) {
            return Lsm6ds3Probe{.addr = addr, .who_am_i = *who_am_i};
        }
        if (error == I2cError::nack) {
            error = who_am_i.error();
        }
    }
    return std::unexpected(error);
}

std::expected<void, I2cError> lsm6ds3_init(std::uint8_t addr) {
    // BDU first: an output register pair is never half old, half new.
    if (auto r = i2c_write_regs(addr, reg_ctrl3_c, {ctrl3_bdu | ctrl3_if_inc}); !r) {
        return r;
    }
    // CTRL1_XL, CTRL2_G. Accel anti-aliasing bandwidth follows the ODR (XL_BW_SCAL_ODR = 0).
    // INT1 push-pull, active high (CTRL3_C PP_OD = H_LACTIVE = 0).
    if (auto r = i2c_write_regs(addr, reg_int1_ctrl, {int1_drdy_g}); !r) {
        return r;
    }
    return i2c_write_regs(addr, reg_ctrl1_xl, {lsm6ds3_ctrl1_xl, lsm6ds3_ctrl2_g});
}

std::expected<std::optional<Lsm6ds3Sample>, I2cError> lsm6ds3_read(std::uint8_t addr) {
    const auto status = i2c_read_reg(addr, reg_status);
    if (!status) {
        return std::unexpected(status.error());
    }
    constexpr std::uint8_t both = status_xlda | status_gda;
    if ((*status & both) != both) {
        return std::nullopt;
    }
    std::array<std::uint8_t, lsm6ds3_sample_bytes> raw{};
    if (auto r = i2c_read_regs(addr, reg_outx_l_g, raw); !r) {
        return std::unexpected(r.error());
    }
    return lsm6ds3_decode(raw);
}

} // namespace hal
