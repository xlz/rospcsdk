#!/bin/sh
# This assumes a complete Windows installation image: disk-pcsdk.qcow2
# This script might need some extra tweaking for your specifc environment
cd ${0%/*}
set -- disk-pcsdk.qcow2 -drive file=exe.fat,id=hda,if=none,cache=none,aio=native -device usb-storage,id=usb-hda,drive=hda,removable=on -serial file:/tmp/qemu.log -snapshot

SPICE_PORT=5900
MACADDR=52:54:00:12:34:57

IMAGE="$1" && shift

export QEMU_AUDIO_DRV=alsa
export QEMU_AUDIO_ADC_TRY_POLL=0
export QEMU_AUDIO_DAC_TRY_POLL=0

QEMU_OPTS="
 -nodefconfig
 -nodefaults
 -enable-kvm
 -daemonize
 -S

 -rtc base=localtime

 -cpu host -smp 2,sockets=1,cores=2,threads=1

 -m 2048

 -vga qxl

 -netdev tap,ifname=qemutap0,script=no,downscript=no,id=tapnet0
  -device virtio-net-pci,id=eth0,netdev=tapnet0,mac=$MACADDR,bus=pci.0,addr=0x3

 -device intel-hda,id=sound0,bus=pci.0,addr=0x4
  -device hda-duplex,id=sound0-codec0,cad=0,bus=sound0.0

 -spice port=$SPICE_PORT,disable-ticketing

 -chardev spicevmc,id=spice0,name=vdagent
  -device virtio-serial-pci,id=ser,bus=pci.0,addr=0x5
   -device virtserialport,id=port0,chardev=spice0,name=com.redhat.spice.0,nr=1,bus=ser.0

 -drive file=$IMAGE,id=vda,if=none,cache=none,aio=native
  -device virtio-blk-pci,id=blk0,scsi=off,drive=vda,bus=pci.0,addr=0x6

  -device ich9-usb-ehci1,id=usb,bus=pci.0,addr=0x7.0x7
   -device ich9-usb-uhci1,masterbus=usb.0,firstport=0,multifunction=on,bus=pci.0,addr=0x7
   -device ich9-usb-uhci2,masterbus=usb.0,firstport=2,bus=pci.0,addr=0x7.0x1
   -device ich9-usb-uhci3,masterbus=usb.0,firstport=4,bus=pci.0,addr=0x7.0x2

 -chardev pipe,id=pipe0,path=monitor
  -mon id=monitor,chardev=pipe0,default
"

set -e
for dir in in out;
	[ -p monitor.$dir ] || mkfifo monitor.$dir
end
qemu-system-x86_64 $QEMU_OPTS "$@" &
echo cont >monitor.in
echo "spicec --host `hostname` --port $SPICE_PORT &"
