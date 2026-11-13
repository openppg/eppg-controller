//! Existing Espressif BLE OTA framing and deterministic sector accumulator.
pub const SECTOR_SIZE:usize=4096;
pub fn crc16(data:&[u8])->u16{let mut crc=0u16;for b in data{crc^=(*b as u16)<<8;for _ in 0..8{crc=if crc&0x8000!=0{(crc<<1)^0x1021}else{crc<<1};}}crc}
#[derive(Clone,Copy,Debug,PartialEq,Eq)]pub enum Command{Start(u32),End}
pub fn parse_command(data:&[u8])->Option<Command>{if data.len()<20||crc16(&data[..18])!=u16::from_le_bytes(data[18..20].try_into().ok()?){return None;}match u16::from_le_bytes(data[..2].try_into().ok()?){1=>Some(Command::Start(u32::from_le_bytes(data[2..6].try_into().ok()?))),2=>Some(Command::End),_=>None}}
pub fn command_ack(command:u16,status:u16)->[u8;20]{let mut b=[0;20];b[..2].copy_from_slice(&3u16.to_le_bytes());b[2..4].copy_from_slice(&command.to_le_bytes());b[4..6].copy_from_slice(&status.to_le_bytes());let crc=crc16(&b[..18]);b[18..].copy_from_slice(&crc.to_le_bytes());b}
pub fn sector_ack(sector:u16,status:u16,expected:u16)->[u8;6]{let mut b=[0;6];b[..2].copy_from_slice(&sector.to_le_bytes());b[2..4].copy_from_slice(&status.to_le_bytes());b[4..].copy_from_slice(&expected.to_le_bytes());b}
#[derive(Clone,Copy,Debug,PartialEq,Eq)]pub enum Receive{Buffered,Complete(usize),Rejected{sector:u16,status:u16,expected:u16},Ignored}
pub struct Sectors{buffer:[u8;SECTOR_SIZE],len:usize,pub sector:u16,pub received:u32,pub total:u32,pub last_activity:u32}
impl Default for Sectors{fn default()->Self{Self{buffer:[0;SECTOR_SIZE],len:0,sector:0,received:0,total:0,last_activity:0}}}
impl Sectors{
    pub fn start(&mut self,total:u32,now:u32){self.len=0;self.sector=0;self.received=0;self.total=total;self.last_activity=now;}
    pub fn data(&self,len:usize)->&[u8]{&self.buffer[..len.min(SECTOR_SIZE)]}
    pub fn receive(&mut self,data:&[u8],now:u32)->Receive{
        if data.len()<3{return Receive::Ignored;}self.last_activity=now;
        let sector=u16::from_le_bytes([data[0],data[1]]);let last=data[2]==255;
        if sector!=self.sector{self.len=0;return Receive::Rejected{sector,status:2,expected:self.sector};}
        let payload=&data[3..];if last&&payload.len()<2{return Receive::Ignored;}
        let n=payload.len()-if last{2}else{0};
        if self.len+n>SECTOR_SIZE{self.len=0;return Receive::Rejected{sector,status:3,expected:0};}
        self.buffer[self.len..self.len+n].copy_from_slice(&payload[..n]);self.len+=n;
        if !last{return Receive::Buffered;}
        if crc16(&self.buffer[..self.len])!=u16::from_le_bytes([payload[n],payload[n+1]]){self.len=0;return Receive::Rejected{sector,status:1,expected:0};}
        Receive::Complete(self.len)
    }
    /// Call only after flash storage accepted the complete sector. An error aborts the session.
    pub fn written(&mut self,len:usize){self.received=self.received.saturating_add(len as u32);self.sector=self.sector.wrapping_add(1);self.len=0;}
    pub fn timed_out(&self,now:u32)->bool{now.wrapping_sub(self.last_activity)>30000}
}
#[cfg(test)]mod tests{use super::*;#[test]fn known_crc(){assert_eq!(crc16(b"123456789"),0x31c3);}#[test]fn crc_retry_and_written(){let mut s=Sectors::default();s.start(3,100);assert_eq!(s.receive(&[0,0,0,1,2],200),Receive::Buffered);let crc=crc16(&[1,2,3]);assert_eq!(s.receive(&[0,0,255,3,crc as u8,(crc>>8) as u8],300),Receive::Complete(3));assert_eq!(s.data(3),[1,2,3]);s.written(3);assert_eq!(s.received,3);assert_eq!(s.sector,1);assert!(s.timed_out(30301));}#[test]fn ack_abi(){assert_eq!(sector_ack(2,1,3),[2,0,1,0,3,0]);assert_eq!(crc16(&command_ack(1,0)[..18]),u16::from_le_bytes(command_ack(1,0)[18..].try_into().unwrap()));}}
