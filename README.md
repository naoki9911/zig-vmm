# zig-vmm

Virtual Machine Monitor for Linux KVM written in Zig.

This software is in early development.
This repository is an experimental implementation and is not intended for production use.

# Acknowledgment
[gokvm](https://github.com/bobuhiro11/gokvm) provides me with a clear implementation example and many useful insights.
Thank you so much!

# Features
- virtio-console
- virtio-net
- virtio-blk

# How to run
[This page](https://zig-vmm.pibvt.net/) is provided with Ubuntu 22.10 running on `zig-vmm`.

## Build Ubuntu RootFS
Run `create_ubuntu_rootfs.sh` to build ubuntu 22.10 rootFS.
```console
$ ./create_ubuntu_rootfs.sh
```

RootFS disk image(`ubuntu2210-base.img`), initramfs(`initrd.img-5.19.0-1018-kvm`) and kernel(`vmlinuz-5.19.0-1018-kvm`) are created.

## Build zig-vmm
Build `zig-vmm` with the latest [zig(master/HEAD)](https://github.com/ziglang/zig).
```console
$ zig version
0.11.0-dev.1930+a097779b6
$ zig build
```

If `zig-vmm` is successfully built, `zig-out/bin/zig-vmm` exists.

## Run zig-vmm
Run `zig-vmm` with sudo.
```console
$ sudo ./zig-out/bin/zig-vmm

INFO  [2023-03-13 05:01:29 9025]: KVM_GET_API_VERSION=12
INFO  [2023-03-13 05:01:29 9025]: KVM_CAP_USER_MEMORY is available
INFO  [2023-03-13 05:01:29 9025]: opened ./ubuntu2210-base.img(size=10737418240 sector_num=20971520)
INFO  [2023-03-13 05:01:29 9025]: waiting console client connection...
```

It waits for the console client connection.
In an another terminal, run console client.
```console
$ sudo ./zig-out/bin/zig-vmm console

INFO  [2023-03-13 05:02:36 9104]: starting console
INFO  [2023-03-13 05:02:36 9104]: configured terminal to raw mode
INFO  [2023-03-13 05:02:36 9104]: connected to zig-vmm!
virtio_blk virtio2: 1/0/0 default/read/poll queues
virtio_blk virtio2: [vda] 20971520 512-byte logical blocks (10.7 GB/10.0 GiB)
Loading iSCSI transport class v2.0-870.
iscsi: registered transport (tcp)

...

[  OK  ] Started LSB: automatic crash report generation.
[  OK  ] Finished Terminate Plymouth Boot Screen.
[  OK  ] Started Serial Getty on hvc0.
[  OK  ] Created slice Slice /system/getty.
[  OK  ] Reached target Login Prompts.

Ubuntu 22.10 ubuntu hvc0

ubuntu login:
```

You can login with user `ubuntu` and password you configured.
If you want to exit the console, type `Ctrl+a` then `x`.

## Network
`zig-vmm` creates tap device for virtio-net.
The tap device is assigned IPv4 address `192.168.200.1/24`.
The disk image is configured to assign `192.168.200.2/24` to the virtio-net NIC.

If you want to connect to the internet from `zig-vmm`'s guest,
please configure IPMasquerade.

```console
sudo iptables -t nat -A POSTROUTING -s 192.168.200.0/24 -o [outgoing interface] -j MASQUERADE
```