#!/bin/bash

set -e

zig build
sudo ./zig-out/bin/zig-vmm
