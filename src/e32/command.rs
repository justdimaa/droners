use serde::{Deserialize, Serialize};

#[derive(Debug, Serialize, Deserialize)]
pub enum Command {
    Controller {
        left_thumb_x: u16,
        left_thumb_y: u16,
        left_trigger: u8,
        right_thumb_x: u16,
        right_thumb_y: u16,
        right_trigger: u8,
        buttons: u16,
    },
}

impl Command {
    pub fn encode(&self, buf: &mut [u8]) {
        serde_json_core::to_slice(self, buf).unwrap();
    }

    pub fn decode(buf: &[u8]) -> Option<Command> {
        let cmd = serde_json_core::from_slice::<Command>(buf);

        match cmd {
            Ok((cmd, _)) => Some(cmd),
            Err(_) => None,
        }
    }
}
