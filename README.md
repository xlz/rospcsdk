ROS driver of Intel PC SDK in VM
================================

A ROS driver exposing features from Intel PC SDK via ROS topics.

This driver runs in a guest Windows VM with PCSDK installed and Ubuntu (Precise) host with ROS groovy on PR2.

Cross compiling with MinGW/MXE on Linux host.

How to build
------------

1. Get Intel PC SDK header files

   Download the Intel PC SDK, install/extract it (probably with Wine), place the `PCSDK` directory here. (Because these source files are not redistributable.)

2. Compile

        cd build
        make

Then there will be a `recog.exe` windows binary somewhere inside.


Deployment
----------

### Choice of VM

QEMU/KVM is well supported by Linux kernel and user space. But PR2 custom kernel does not enable KVM. Custom built kernel with KVM enabled is required.

VirtualBox kernel drivers still taint and unstabilize the kernel.

VMware Player freezes the whole system upon certain network activity, and does not work well with ALSA.

### Network

NAT with forwarding won't work because ROS listens on ephemeral ports. Bridged network is recommended. Tap/host-only is good for local testing purposes.

### Serial port

This proxy uses COM1 as stdout and stderr. Configure the baudrate of COM1 to something high like 115200.

### Installation

Install PCSDK in VM. Place *.exe and run.cmd (`examples/run.cmd`, review and edit the file first) in it, and set autostart for run.cmd. An example script to start up a QEMU VM is in `examples/pcsdk-qemu.sh`.


Usage
-----

After the VM is running and the driver node starts, this driver provides ROS topics and services:

* `/pcsdk/recog/speech`, topic, publish recognized speech. Type: `std_msgs/String`.

        rostopic echo /pcsdk/recog/speech

* `/pcsdk/recog/alert`, topic, publish problems with the audio signal. Type: `std_msgs/String`, value within VOLUME_HIGH, VOLUME_LOW, SNR_LOW, SPEECH_UNRECOGNIZABLE.
  
        rostopic echo /pcsdk/recog/alert

* `/pcsdk/recog/stop`, service, stop speech recognition and wasting cpu cycles.
  
        rosservice call /pcsdk/recog/stop

* `/pcsdk/recog/dictate`, service, start dictation mode - returns whatever it hears.
  
        rosservice call /pcsdk/recog/dictate

* `/pcsdk/recog/grammar`, Service, start grammar mode - return only one of the parameter phrases. Type: `pcsdk/Grammar`. `pcsdk/Grammar` message needs to be installed system-wide somehow.

* `/pcsdk/synth`, topic, subscribes to sentences to synthesize. Type: `std_msgs/String`.
  
        rostopic pub /pcsdk/synth std_msgs/String "hello" --once

