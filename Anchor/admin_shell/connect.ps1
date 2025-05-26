param(
    [Parameter(Mandatory=$true)]
    [string]$HostName
)

# Key-Pfad relativ zum aktuellen Verzeichnis
$keyPath = ".\.ssh\anchor_admin_id"
$user = "anchor"

# SSH-Befehl ausführen
ssh -i $keyPath "$user@$HostName"

