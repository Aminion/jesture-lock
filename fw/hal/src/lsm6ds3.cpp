#include "hal/lsm6ds3.hpp"

#include <array>
#include <span>

namespace {

// Same addresses and the bits used here on LSM6DS3 (DocID026899) and LSM6DS3TR-C.
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
constexpr std::uint8_t status_xlda = 1U << 0U;
constexpr std::uint8_t status_gda = 1U << 1U;

constexpr std::array<std::uint8_t, 2> addresses{0x6B, 0x6A};

} // namespace

namespace hal {

std::expected<Lsm6ds3Probe, I2cError> lsm6ds3_probe() {
    I2cError error = I2cError::nack;
    for (const std::uint8_t addr : addresses) {
        std::uint8_t who_am_i = 0;
        const auto r = i2c_read_regs(addr, reg_who_am_i, std::span(&who_am_i, 1));
        if (r) {
            return Lsm6ds3Probe{.addr = addr, .who_am_i = who_am_i};
        }
        if (error == I2cError::nack) {
            error = r.error();
        }
    }
    return std::unexpected(error);
}

std::expected<void, I2cError> lsm6ds3_init(std::uint8_t addr) {
    // BDU first: an output register pair is never half old, half new.
    const std::uint8_t ctrl3 = ctrl3_bdu | ctrl3_if_inc;
    if (auto r = i2c_write_regs(addr, reg_ctrl3_c, std::span(&ctrl3, 1)); !r) {
        return r;
    }
    // CTRL1_XL, CTRL2_G. Accel anti-aliasing bandwidth follows the ODR (XL_BW_SCAL_ODR = 0).
    const std::array<std::uint8_t, 2> ctrl{odr_104hz | fs_xl_2g, odr_104hz | fs_g_250dps};
    return i2c_write_regs(addr, reg_ctrl1_xl, ctrl);
}

std::expected<std::optional<Lsm6ds3Sample>, I2cError> lsm6ds3_read(std::uint8_t addr) {
    std::uint8_t status = 0;
    if (auto r = i2c_read_regs(addr, reg_status, std::span(&status, 1)); !r) {
        return std::unexpected(r.error());
    }
    constexpr std::uint8_t both = status_xlda | status_gda;
    if ((status & both) != both) {
        return std::nullopt;
    }
    std::array<std::uint8_t, lsm6ds3_sample_bytes> raw{};
    if (auto r = i2c_read_regs(addr, reg_outx_l_g, raw); !r) {
        return std::unexpected(r.error());
    }
    return lsm6ds3_decode(raw);
}

} // namespace hal
