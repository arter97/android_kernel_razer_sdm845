#!/sbin/sh

mount -o ro /dev/block/bootdevice/by-name/vendor$(cat /proc/cmdline | tr ' ' '\n' | grep slot_suffix | tr '=' ' ' | awk '{print $2}') /vendor
if cat /vendor/etc/fstab.qcom | grep -q /data; then
  # userdata is f2fs
  mv /etc/twrp.fstab.f2fs /etc/twrp.fstab
else
  mv /etc/twrp.fstab.stock /etc/twrp.fstab
fi

umount /vendor
touch /dev/fstab_ready
