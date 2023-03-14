#!/bin/bash

set -e

zig build
sudo ./zig-out/bin/zig-vmm --initramfs initrd.img-5.19.0-1018-kvm --vmlinuz vmlinuz-5.19.0-1018-kvm --disk_img ubuntu2210-base.img