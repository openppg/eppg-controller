#!/usr/bin/env python3
"""Export Rust app-only OTA and full recovery artifacts; validate linked scope."""
import argparse,json,os,re,shutil,struct,subprocess,sys
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('release',type=Path);p.add_argument('--profile',choices=['physical','qemu'],default='physical');args=p.parse_args()
release=args.release.resolve();sdk=next(release.glob('build/esp-idf-sys-*/out/build'))
out=release/'images';out.mkdir(exist_ok=True);elf=release/'firmware-sp140';app=out/'firmware.bin'
python=os.environ.get('PYTHON',sys.executable)
subprocess.run([python,'-m','esptool','--chip','esp32s3','elf2image','--flash_mode','dio' if args.profile=='qemu' else 'qio','--flash_freq','80m','--flash_size','8MB','-o',str(app),str(elf)],check=True)
content=app.read_bytes();assert len(content)<=0x330000,'OTA slot exceeded';assert content.count(b'OPPGFWV1')==1,'retained marker missing/duplicated'
version=struct.unpack_from('<BBI',content,content.index(b'OPPGFWV1')+8)
link=sdk/'libespidf.map';text=link.read_text()
assert not re.search(r'framework-arduino|libarduino|ArduinoJson|Adafruit_|Preferences::|NimBLE-Arduino',text,re.I),'Arduino dependency linked'
assert not re.search(r'[/\\]src[/\\]sp140[/\\].*\.cpp',text),'legacy C++ application linked'
partitions=sdk/'partition_table/partition-table.bin';actual={};expected={'nvs':(0x9000,0x5000),'otadata':(0xe000,0x2000),'app0':(0x10000,0x330000),'app1':(0x340000,0x330000),'spiffs':(0x670000,0x180000),'coredump':(0x7f0000,0x10000)}
for n in range(0,len(partitions.read_bytes()),32):
 magic,kind,subtype,start,length,label,flags=struct.unpack_from('<HBBII16sI',partitions.read_bytes(),n)
 if magic!=0x50aa:break
 actual[label.rstrip(b'\0').decode()]=(start,length)
assert actual==expected,f'partition layout changed: {actual}'
boot=sdk/'bootloader/bootloader.bin';ota=sdk/'ota_data_initial.bin'
assert boot.exists() and ota.exists(),'bootloader/initial OTA metadata missing'
subprocess.run([python,'-m','esptool','--chip','esp32s3','merge_bin','--flash_mode','dio' if args.profile=='qemu' else 'qio','--flash_freq','80m','--flash_size','8MB','-o',str(out/'recovery.bin'),'0',str(boot),'0x8000',str(partitions),'0xe000',str(ota),'0x10000',str(app)],check=True)
for source,target in [(elf,'firmware.elf'),(link,'firmware.map'),(boot,'bootloader.bin'),(partitions,'partitions.bin')]:shutil.copy2(source,out/target)
if args.profile=='qemu':
 data=(out/'recovery.bin').read_bytes();(out/'qemu-flash.bin').write_bytes(data+b'\xff'*(8*1024*1024-len(data)))
report={'profile':args.profile,'version':f'{version[0]}.{version[1]}','build':version[2],'app_bytes':len(content),'ota_slot_bytes':0x330000,'arduino_objects':False,'legacy_cpp_app_objects':False,'partition_layout':'installed-8MB','elf':str(elf),'images':str(out)}
(out/'release-check.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
