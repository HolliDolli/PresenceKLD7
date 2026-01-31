while (1) {
    echo $((Get-Date).ToString('dd-MM-yyyy hh:mm:ss'))
    if (Test-Connection -TargetName 192.168.2.209 -Quiet) { esphome run TestLolin-V01.yaml }
}