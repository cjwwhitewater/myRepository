#!/usr/bin/fish


# 以下是和具体任务相关的内容
set serverName "hw"
set PortNumberOfTargetMachine 8300
set sshCommand ssh -oPort=$PortNumberOfTargetMachine
set targetDirectory /u/drone/dcontroller

# create directories
$sshCommand zb@$serverName mkdir -p  $targetDirectory/bin

# copy binaries and associated files to the target machine.
rsync -e 'ssh -oPort=8300'  -avz  bin/*    zb@$serverName:$targetDirectory/bin
rsync -e 'ssh -oPort=8300'  -r config      zb@$serverName:$targetDirectory
rsync -e 'ssh -oPort=8300'  run            zb@$serverName:$targetDirectory



