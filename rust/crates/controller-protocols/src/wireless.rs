//! ESP-NOW v1 application protocol, independent of radio security.
pub const MAGIC:u16=0x5050;pub const VERSION:u8=1;
pub fn crc32(data:&[u8])->u32{let mut crc=u32::MAX;for b in data {crc^=*b as u32;for _ in 0..8{crc=(crc>>1)^(0xedb88320u32&0u32.wrapping_sub(crc&1));}}!crc}
pub fn sequence_newer(candidate:u32,previous:u32)->bool{candidate.wrapping_sub(previous) as i32>0}
#[derive(Clone,Copy,Debug,Default,PartialEq,Eq)]
pub struct Control{pub session:u32,pub sequence:u32,pub uptime:u32,pub throttle:u16,pub state:u8,pub mode:u8,pub flags:u8,pub reserved:u8}
impl Control{
    pub fn encode(self)->[u8;26]{let mut b=[0;26];b[..2].copy_from_slice(&MAGIC.to_le_bytes());b[2]=VERSION;b[3]=1;b[4..8].copy_from_slice(&self.session.to_le_bytes());b[8..12].copy_from_slice(&self.sequence.to_le_bytes());b[12..16].copy_from_slice(&self.uptime.to_le_bytes());b[16..18].copy_from_slice(&self.throttle.to_le_bytes());b[18]=self.state;b[19]=self.mode;b[20]=self.flags;b[21]=self.reserved;let crc=crc32(&b[..22]);b[22..].copy_from_slice(&crc.to_le_bytes());b}
    pub fn decode(b:&[u8])->Option<Self>{if b.len()!=26||!header(b,1)||b[18]>2||b[19]>1{return None;}let throttle=u16::from_le_bytes(b[16..18].try_into().ok()?);if throttle>4095{return None;}Some(Self{session:u32::from_le_bytes(b[4..8].try_into().ok()?),sequence:u32::from_le_bytes(b[8..12].try_into().ok()?),uptime:u32::from_le_bytes(b[12..16].try_into().ok()?),throttle,state:b[18],mode:b[19],flags:b[20],reserved:b[21]})}
}
#[derive(Clone,Copy,Debug,Default,PartialEq,Eq)]
pub struct Ack{pub session:u32,pub sequence:u32,pub uptime:u32,pub state:u8,pub flags:u8,pub age:u16}
impl Ack{
    pub const LINK_READY:u8=1;pub const FAILSAFE:u8=2;pub const ACCEPTED:u8=4;
    pub fn encode(self)->[u8;24]{let mut b=[0;24];b[..2].copy_from_slice(&MAGIC.to_le_bytes());b[2]=VERSION;b[3]=2;b[4..8].copy_from_slice(&self.session.to_le_bytes());b[8..12].copy_from_slice(&self.sequence.to_le_bytes());b[12..16].copy_from_slice(&self.uptime.to_le_bytes());b[16]=self.state;b[17]=self.flags;b[18..20].copy_from_slice(&self.age.to_le_bytes());let crc=crc32(&b[..20]);b[20..].copy_from_slice(&crc.to_le_bytes());b}
    pub fn decode(b:&[u8])->Option<Self>{if b.len()!=24||!header(b,2)||b[16]>2{return None;}Some(Self{session:u32::from_le_bytes(b[4..8].try_into().ok()?),sequence:u32::from_le_bytes(b[8..12].try_into().ok()?),uptime:u32::from_le_bytes(b[12..16].try_into().ok()?),state:b[16],flags:b[17],age:u16::from_le_bytes(b[18..20].try_into().ok()?)})}
}
fn header(b:&[u8],kind:u8)->bool {b.len()>=8&&b[..2]==MAGIC.to_le_bytes()&&b[2]==VERSION&&b[3]==kind&&crc32(&b[..b.len()-4])==u32::from_le_bytes(b[b.len()-4..].try_into().unwrap_or([0;4]))}
#[cfg(test)]mod tests{use super::*;#[test]fn crc_known(){assert_eq!(crc32(b"123456789"),0xcbf43926);}#[test]fn control_roundtrip(){let c=Control{session:123,sequence:456,uptime:789,throttle:2048,state:1,mode:1,..Control::default()};assert_eq!(Control::decode(&c.encode()),Some(c));let mut bad=c.encode();bad[9]^=1;assert!(Control::decode(&bad).is_none());}#[test]fn sequence_wrap(){assert!(sequence_newer(0,u32::MAX));assert!(!sequence_newer(10,10));assert!(!sequence_newer(9,10));}#[test]fn ack_roundtrip(){let a=Ack{session:123,sequence:456,uptime:789,state:2,flags:7,age:150};assert_eq!(Ack::decode(&a.encode()),Some(a));}}
