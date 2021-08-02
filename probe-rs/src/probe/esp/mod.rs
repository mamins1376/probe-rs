use crate::*;
use crate::architecture::xtensa::communication_interface::XtensaProbeInterface;

use serial::{*, core::SerialDevice};
use slip_codec::{Decoder, Encoder};

use enum_primitive_derive::Primitive;
use num_traits::{FromPrimitive, ToPrimitive};

use std::borrow::Cow;
use std::env::var;
use std::fmt::Debug;
use std::fs::{OpenOptions, read_dir};
use std::result::Result;
use std::time::Duration;

type IoError = std::io::Error;

type ProbeResult<T> = Result<T, DebugProbeError>;

pub(crate) fn list_esp_devices() -> impl Iterator<Item=DebugProbeInfo> {
    if let Ok(ttys) = var("PROBE_ESP_TTY") {
        ttys.split(':')
            .map(From::from)
            .collect()
    } else if cfg!(windows) {
        Vec::new()
    } else {
        let mut options = OpenOptions::new();
        options.read(true);
        options.write(true);

        read_dir(r"/dev")
            .into_iter()
            .flatten()
            .filter_map(|f| f.ok())
            .map(|f| f.path())
            .filter(|p| p.to_str().unwrap().starts_with(r"/dev/ttyUSB"))
            .filter(|p| options.open(p).is_ok())
            .filter_map(|p| p.to_str().map(|s| s.to_owned()))
            .collect()
    }.into_iter().map(|id| DebugProbeInfo {
        identifier: id.clone(),
        vendor_id: 0,
        product_id: 0,
        serial_number: id.into(),
        probe_type: DebugProbeType::Esp,
        hid_interface: None,
    })
}

pub(crate) struct Esp {
    tty: String,
    interface: Option<EspInterface>,
}

impl Esp {
    const VENDOR_ID: u16 = 0;
    const PRODUCT_ID: u16 = 0;
}

impl DebugProbe for Esp {
    fn attach(&mut self) -> ProbeResult<()> {
        Ok(if self.interface.is_none() {
            let mut interface = EspInterface::new(&self.tty)
                .map_err(EspInterface::to_probe_error)?;
            interface.attach()?;
            self.interface = Some(interface)
        })
    }

    fn detach(&mut self) -> ProbeResult<()> {
        Ok(if let Some(interface) = self.interface.as_mut() {
            interface.detach()?
        })
    }

    fn get_name(&self) -> &str {
        todo!()
    }

    fn into_probe(self: Box<Self>) -> Box<dyn DebugProbe> {
        self as _
    }

    fn new_from_selector(selector: impl Into<DebugProbeSelector>) -> ProbeResult<Box<Self>>
    where Self: Sized {
        let DebugProbeSelector {
            vendor_id,
            product_id,
            serial_number,
        } = selector.into();

        let tty = match (vendor_id == Self::VENDOR_ID && product_id == Self::PRODUCT_ID, serial_number) {
            (true, Some(tty)) => tty,
            _ => Err(ProbeCreationError::NotFound)?,
        };

        Ok(Self {
            tty,
            interface: None,
        }.into())
    }

    fn select_protocol(&mut self, protocol: WireProtocol) -> ProbeResult<()> {
        match protocol {
            WireProtocol::Esp => Ok(()),
            _ => Err(DebugProbeError::UnsupportedProtocol(protocol)),
        }
    }

    fn set_speed(&mut self, speed_khz: u32) -> ProbeResult<u32> {
        match speed_khz {
            115 => Ok(speed_khz),
            _ => Err(DebugProbeError::UnsupportedSpeed(speed_khz)),
        }
    }

    fn speed(&self) -> u32 {
        115
    }

    fn target_reset(&mut self) -> ProbeResult<()> {
        todo!()
    }

    fn target_reset_assert(&mut self) -> ProbeResult<()> {
        todo!()
    }

    fn target_reset_deassert(&mut self) -> ProbeResult<()> {
        todo!()
    }

    fn has_xtensa_interface(&self) -> bool {
        true
    }

    fn try_get_xtensa_interface(self: Box<Self>) -> Result<XtensaProbeInterface, (Box<dyn DebugProbe>, DebugProbeError)> {
        todo!()
    }
}

impl Debug for Esp {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        "Esp".fmt(f)
    }
}

struct EspInterface {
    serial: SystemPort,
    encoder: Encoder,
    decoder: Decoder,
    buffer: Vec<u8>,
}

impl EspInterface {
    const DEFAULT_TIMEOUT: Duration = Duration::from_secs(3);
    const SYNC_TIMEOUT: Duration = Duration::from_millis(100);

    fn new(tty: impl AsRef<str>) -> serial::core::Result<Self> {
        Ok(Self {
            serial: open(tty.as_ref())?,
            decoder: Decoder::new(),
            encoder: Encoder::new(),
            buffer: Vec::new(),
        })
    }

    fn attach(&mut self) -> ProbeResult<()> {
        self.set_baud(BaudRate::Baud115200)?;
        if self.get_baud()? != 115200 {
            Err(Self::error("Cannot set the baud rate to 115200"))?
        }

        let ref sync = Command::sync().serialize();

        self.set_timeout(Self::SYNC_TIMEOUT)?;

        for _ in 0..7 {
            self.send(sync)?;

            match self.read() {
                Some(error) => error?,
                None => continue,
            };

            match Command::parse(&self.buffer) {
                Ok(resp) => match resp.opcode {
                    Opcode::Sync => break,
                    _ => Err(Self::error("Invalid opcode after sync request"))?,
                },
                Err(error) => Err(DebugProbeError::Other(error.into()))?,
            }
        }

        if self.buffer.is_empty() {
            Err(Self::error("Device did not respond in time"))?
        }
        self.buffer.clear();

        // drain any leftover bytes - esp8266 will usually send multiple Syncs
        loop {
            match self.read() {
                Some(error) => error?,
                None => break,
            };
        }
        self.buffer.clear();
        self.set_timeout(Self::DEFAULT_TIMEOUT)?;

        Ok(())
    }

    fn detach(&mut self) -> ProbeResult<()> {
        todo!()
    }

    fn read(&mut self) -> Option<ProbeResult<usize>> {
        match self.decoder.decode(&mut self.serial, &mut self.buffer) {
            Ok(len) => Some(Ok(len)),
            Err(slip_codec::Error::ReadError(e)) => match e.kind() {
                std::io::ErrorKind::TimedOut => None,
                _ => Some(Err(DebugProbeError::Io(e))),
            },
            Err(_) => Some(Err(Self::error("Invalid SLIP packet recevied").into())),
        }
    }

    fn send(&mut self, buf: impl AsRef<[u8]>) -> ProbeResult<()> {
        Ok(self.encoder.encode(buf.as_ref(), &mut self.serial).map(drop)?)
    }

    fn set_timeout(&mut self, timeout: Duration) -> ProbeResult<()> {
        Ok(SerialDevice::set_timeout(&mut self.serial, timeout)
            .map_err(Self::to_probe_error)?)
    }

    fn set_baud(&mut self, baud: BaudRate) -> ProbeResult<()> {
        self.serial
            .reconfigure(&|s| s.set_baud_rate(baud))
            .map_err(|_| DebugProbeError::UnsupportedSpeed(baud.speed() as u32 / 1000))
    }

    fn get_baud(&self) -> ProbeResult<usize> {
        self.serial.read_settings()
            .map_err(Self::to_probe_error)?
            .baud_rate()
            .map(|s| s.speed())
            .ok_or(Self::error("Cannot read baud rate back").into())
    }

    fn to_probe_error(error: serial::Error) -> ProbeCreationError {
        use serial::ErrorKind::*;

        match error.kind() {
            NoDevice | InvalidInput => ProbeCreationError::CouldNotOpen,
            Io(error) => Self::error(IoError::from(error)),
        }
    }

    fn error(e: impl Into<Box<dyn std::error::Error + Send + Sync>>) -> ProbeCreationError {
        ProbeCreationError::ProbeSpecific(e.into())
    }
}

#[derive(thiserror::Error, Debug)]
enum ReceiveError {
    #[error("Malformed SLIP packet or unreadable port")]
    Io(#[from] IoError),
    #[error("Malformed bootloader packet")]
    Data(#[from] InvalidResponse),
}

#[derive(PartialEq, Eq, Debug)]
struct Command<'a> {
    opcode: Opcode,
    data: Cow<'a, [u8]>,
}

impl<'a> Command<'a> {
    fn new(opcode: Opcode, data: impl Into<Cow<'a, [u8]>>) -> Self {
        Self {
            opcode,
            data: data.into(),
        }
    }

    fn parse(b: &'a [u8]) -> Result<Self, InvalidResponse> {
        use self::InvalidResponse::*;

        (b.len() > 8).then(|| ()).ok_or(Incomplete)?;
        (b[0] == 1).then(|| ()).ok_or(InvalidDirection)?;
        let opcode = Opcode::from_u8(b[1]).ok_or(UnknownOpcode(b[1]))?;

        let mut size = [0; 2];
        size.copy_from_slice(&b[2..4]);
        let size = u16::from_le_bytes(size);

        (b.len() == (8 + size as usize)).then(|| ())
            .ok_or(SizeMismatch(size).into())?;

        // use data field for read_reg response
        let data = if opcode == Opcode::ReadReg && size == 0 {
            &b[4..8]
        } else {
            &b[8..]
        };

        Ok(Self::new(opcode, data))
    }

    fn sync() -> Self {
        let mut data = [0u8; 36];
        data[..4].copy_from_slice(&[0x07, 0x07, 0x12, 0x20]);
        data[4..].iter_mut().for_each(|b| *b = 0x55);
        Self::new(Opcode::Sync, data.to_vec())
    }

    fn serialize(&self) -> Box<[u8]> {
        let data = &self.data;
        let len = data.len();

        let mut b = Vec::with_capacity(8 + len);

        b.push(0);
        b.push(self.opcode.to_u8().unwrap());
        b.extend((len as i16).to_le_bytes());
        b.extend([0; 4]);
        b.extend(data.iter());

        assert!(b.len() == b.capacity());

        b.into()
    }
}

#[derive(thiserror::Error, Debug)]
enum InvalidResponse {
    #[error("Incomplete SLIP packet recevied")]
    Incomplete,
    #[error("Device sent a command with Request direction")]
    InvalidDirection,
    #[error("Unknown opcode: {0}")]
    UnknownOpcode(u8),
    #[error("Size field does not match the data length")]
    SizeMismatch(u16),
}

#[derive(Primitive, Clone, Copy, PartialEq, Eq, Debug)]
enum Opcode {
    FlashBegin = 0x02,
    FlashData  = 0x03,
    FlashEnd   = 0x04,
    MemBegin   = 0x05,
    MemEnd     = 0x06,
    MemData    = 0x07,
    Sync       = 0x08,
    WriteReg   = 0x09,
    ReadReg    = 0x0a,
}
