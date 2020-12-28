#[allow(dead_code)]
#[derive(Debug, Clone, Copy)]
pub enum Register {
    WIA = 0x00,
    INFO = 0x01,
    ST1 = 0x02,
    HXL = 0x03,
    HXH = 0x04,
    HYL = 0x05,
    HYH = 0x06,
    HZL = 0x07,
    HZH = 0x08,
    ST2 = 0x09,
    CNTL = 0x0A,
    RSV = 0x0B,
    ASTC = 0x0C,
    TS1 = 0x0D,
    TS2 = 0x0E,
    I2CDIS = 0x0F,
    ASAX = 0x10,
    ASAY = 0x11,
    ASAZ = 0x12,
}
