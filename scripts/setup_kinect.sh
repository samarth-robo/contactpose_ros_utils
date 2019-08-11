#!/usr/bin/env bash
sysctl -w vm.min_free_kbytes=65536
sh -c 'echo 256 > /sys/module/usbcore/parameters/usbfs_memory_mb'
