qemu-system-x86_64 -kernel ./bzImage -initrd ./initrd -append "root=/dev/ram rw console=hvc0 rdinit=/sbin/init" \
-device virtio-serial-pci,id=virtio-serial0              \
-chardev stdio,id=charconsole0                           \
-device virtconsole,chardev=charconsole0,id=console0 \
-device virtio-net,netdev=network0 -netdev tap,id=network0,ifname=tap0,script=no,downscript=no
