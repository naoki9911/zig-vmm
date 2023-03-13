#!/bin/bash

set -eu

IMAGE_NAME=ubuntu2210-base.img
IMAGE_MOUNT_PATH=/media

if [ -e $IMAGE_NAME ]; then
  echo "image $IMAGE_NAME already exists"
  exit 0
fi

if [ ! -e $IMAGE_MOUNT_PATH ]; then
  echo "$IMAGE_MOUNT_PATH does not exist"
  exit 1
fi

fallocate -l 10G $IMAGE_NAME
mkfs.ext4 $IMAGE_NAME
sudo mount -o loop $IMAGE_NAME $IMAGE_MOUNT_PATH

if [ ! -e ubuntu-22.10-server-cloudimg-amd64-root.tar.xz ]; then
    wget https://cloud-images.ubuntu.com/releases/22.10/release/ubuntu-22.10-server-cloudimg-amd64-root.tar.xz
fi

echo "=== Extracting rootfs from cloudimg ==="
sudo tar -C $IMAGE_MOUNT_PATH -xf ubuntu-22.10-server-cloudimg-amd64-root.tar.xz
echo "Done."

echo "=== Creating User ubuntu ==="
sudo chroot $IMAGE_MOUNT_PATH adduser --add_extra_groups ubuntu
sudo chroot $IMAGE_MOUNT_PATH usermod ubuntu -aG sudo
sudo chroot $IMAGE_MOUNT_PATH dpkg-reconfigure openssh-server

# Remove cloudimg config to enable PasswordAuhtentication
# $ cat 60-cloudimg-settings.conf
# PasswordAuthentication no
#
sudo chroot $IMAGE_MOUNT_PATH rm /etc/ssh/sshd_config.d/60-cloudimg-settings.conf 


sudo chroot $IMAGE_MOUNT_PATH rm /etc/resolv.conf
sudo chroot $IMAGE_MOUNT_PATH /bin/bash -c 'echo "nameserver 8.8.8.8" > /etc/resolv.conf'

sudo chroot $IMAGE_MOUNT_PATH apt update
sudo chroot $IMAGE_MOUNT_PATH apt install -y linux-kvm
sudo chroot $IMAGE_MOUNT_PATH rm /etc/resolv.conf

sudo chroot $IMAGE_MOUNT_PATH /bin/bash -c 'cat << EOF > /etc/netplan/00-config.yaml
network:
  ethernets:
    enp0s2:
      dhcp4: false
      addresses: [192.168.200.2/24]
      gateway4: 192.168.200.1
      nameservers:
        addresses: [8.8.8.8]
  version: 2
EOF'

sudo cp /media/boot/vmlinuz-* .
sudo cp /media/boot/initrd.img-* .

sudo umount $IMAGE_NAME