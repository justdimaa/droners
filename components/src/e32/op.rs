#[derive(Debug, Clone, Copy)]
pub enum OperationMode {
    // M0: Low | M1: Low
    /// Uart and wireless channel are open,
    /// transparent transmission is open.
    Normal = 0,

    // M0: High | M1: Low
    /// Uart and wireless channel are open,
    /// the only diffference with mode 0 is that before transmitting data,
    /// increasing the wake up code automatically,
    /// so that it can awake the receiver under mode 3.
    WakeUp = 1,

    // M0: Low | M1: High
    /// Uart close, wireless is under air-awaken mode,
    /// after receiving data, Uart open and send data.
    PowerSaving = 2,

    // M0: High | M1: High
    /// Sleep mode, receiving parameter setting command is available.
    Sleep = 3,
}

#[derive(Debug, Clone, Copy)]
pub enum OperationCode {
    ReadCfg = 0xCC,
    ReadVersion = 0xC3,
    Reset = 0xC4,
}
