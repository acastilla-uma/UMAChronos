param([string]$Ply = '')
Set-Location -LiteralPath $PSScriptRoot
if ($Ply) { python server.py --ply $Ply } else { python server.py }
