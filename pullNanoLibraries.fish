#!/usr/bin/fish

# Synchronize library files from the target machine into the work machine.

# The following setting are for accessing the nano machine via frp (not LAN)
# CaoJiaWei and ChenYinRui should use the following lines.
set sshCommand     'ssh -oPort=8300'
set targetMachine  '124.70.32.23'

set sysroot '/home/cjw/x-tools/aarch64-nano-linux-gnu/aarch64-nano-linux-gnu/sysroot'
# The following setting are for accessing the nano machine via LAN (not frp)
# ZhangBo can use the following lines.
#set sshCommand     'ssh'
#set targetMachine  'nano'

# Header files.
rsync -e $sshCommand -r --relative -avz cjw@$targetMachine:/usr/include                    $sysroot
rsync -e $sshCommand -r --relative -avz cjw@$targetMachine:/usr/local/include              $sysroot

# Libraries.
# Although currently we only need files and a subdirectory 'aarch64-linux-gnu' of the source directory
# ‘/usr/lib’, we could not figure out command options of rsync for this purpose. Thus, we synchronize 
# all content of this directory. Of course, this operation will pull many libraries which are not 
# used by our current 'drone' project.
rsync -e $sshCommand -r --relative -avz zb@$targetMachine:/usr/lib                      $sysroot
rsync -e $sshCommand -r --relative -avz zb@$targetMachine:/usr/local/lib                $sysroot
rsync -e $sshCommand -r --relative -avz zb@$targetMachine:/lib/aarch64-linux-gnu/       $sysroot

# 小组内部使用的一些库
rsync -e $sshCommand -r --relative -avz zb@$targetMachine:/usrLib  $sysroot

# 处理一些特殊的库
# libc.so，解释参文档
cp specialNanoLibraries/libc.so     $sysroot/lib/aarch64-linux-gnu
cp specialNanoLibraries/libc.so     $sysroot/usr/lib/aarch64-linux-gnu

# libpthread
cd $sysroot/lib/aarch64-linux-gnu
unlink libpthread.so
ln -rs libpthread.so.0 libpthread.so
cd $sysroot/usr/lib/aarch64-linux-gnu
unlink libpthread.so
ln -rs libpthread.so.0 libpthread.so

# dl库
cd $sysroot/lib/aarch64-linux-gnu
unlink libdl.so
ln -rs libdl.so.2 libdl.so

cd $sysroot/usr/lib/aarch64-linux-gnu
unlink libdl.so
ln -rs libdl.so.2 libdl.so
