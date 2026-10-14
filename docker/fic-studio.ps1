<#
.SYNOPSIS
    FIC Studio in Docker: Windows launcher.

.DESCRIPTION
    Double-click fic-studio.cmd, or run in PowerShell:

      .\fic-studio.ps1              Start FIC Studio and open it in your browser
      .\fic-studio.ps1 stop         Stop it (your settings and files are kept)
      .\fic-studio.ps1 restart      Stop, then start again
      .\fic-studio.ps1 update       Download the newest version and restart
      .\fic-studio.ps1 logs         Show what it is doing (Ctrl-C to leave)
      .\fic-studio.ps1 shell        Open a terminal inside it
      .\fic-studio.ps1 status       Is it running, which version, what is saved
      .\fic-studio.ps1 reset        Forget all settings and saved robots
      .\fic-studio.ps1 uninstall    Remove FIC Studio from Docker completely

    Options (for start, restart and update):
      -Dev          Start the Dev Console instead of the Studio
      -Desktop      Start only the desktop
      -Port N       Browser port on this computer (default 6080)
      -WebPort N    Port of the Studio's web remote (default 8088)
      -Lan          Let phones on your network open the Studio's web remote
                    (they sign in as admin, with a password printed at start)
      -Folder DIR   Shared folder (default: FIC_Studio in your user folder)
      -Image REF    Image to run (default ghcr.io/trl-sussex/fic_interface:latest)
      -NoBrowser    Do not open the browser

    Everything the Studio saves is kept in a Docker volume (fic-studio-home);
    recordings, datasets and your own extensions go to the shared folder.

    The desktop gets a new random password every start, which the browser
    link carries, so other users of this computer cannot open it.
#>
[CmdletBinding()]
param(
    [Parameter(Position = 0)]
    [ValidateSet('start', 'stop', 'restart', 'update', 'logs', 'shell', 'status', 'reset', 'uninstall', 'help')]
    [string]$Command = 'start',
    [switch]$Dev,
    [switch]$Desktop,
    [int]$Port = 6080,
    [int]$WebPort = 8088,
    [switch]$Lan,
    [string]$Folder = (Join-Path $env:USERPROFILE 'FIC_Studio'),
    [string]$Image = $(if ($env:FIC_IMAGE) { $env:FIC_IMAGE } else { 'ghcr.io/trl-sussex/fic_interface:latest' }),
    [switch]$NoBrowser
)

$ErrorActionPreference = 'Stop'
# Invoke-WebRequest's progress bar slows it down and garbles the output.
$ProgressPreference = 'SilentlyContinue'
$Name = 'fic-studio'
$Volume = 'fic-studio-home'

function Say([string]$Text) { Write-Host $Text }
function Step([string]$Text) { Write-Host "==> $Text" -ForegroundColor Cyan }
function Warn([string]$Text) { Write-Host "Warning: $Text" -ForegroundColor Yellow }
function Fail([string]$Text) { Write-Host "Error: $Text" -ForegroundColor Red; exit 1 }

# docker, with its output captured (stdout and stderr) and never throwing:
# failures are reported through $LASTEXITCODE.
function Docker-Quiet {
    $previous = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    try { $out = & docker @args 2>&1 } finally { $ErrorActionPreference = $previous }
    return $out
}

# docker, writing straight to the console (so `logs -f` streams and
# `exec -it` gets a terminal). Windows PowerShell (in ISE, or with stderr
# redirected) turns a native command's stderr into errors, which would stop
# the script under 'Stop'. Callers check $LASTEXITCODE.
function Docker-Show {
    $previous = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    try { & docker @args } finally { $ErrorActionPreference = $previous }
}

# The first line of plain text docker printed (skipping stderr records).
function First-Text($Output) {
    return ($Output | Where-Object { $_ -is [string] } | Select-Object -First 1)
}

# Show captured docker output as plain lines (stderr records as their text).
function Show-Output($Output) {
    foreach ($line in $Output) {
        if ($line -is [System.Management.Automation.ErrorRecord]) { $line = $line.Exception.Message }
        $line = "$line"
        # `docker run -d` prints the new container's ID even when it then fails.
        if ($line -notmatch '^[0-9a-f]{64}$') { Write-Host $line }
    }
}

function Ensure-Docker {
    # Docker Desktop installed for one user only (in %LOCALAPPDATA%) does not
    # always put docker.exe on PATH; look in both install locations.
    $desktopDirs = @((Join-Path $env:ProgramFiles 'Docker\Docker'),
                     (Join-Path $env:LOCALAPPDATA 'Programs\DockerDesktop'))
    if (-not (Get-Command docker -ErrorAction SilentlyContinue)) {
        foreach ($dir in $desktopDirs) {
            $bin = Join-Path $dir 'resources\bin'
            if (Test-Path -LiteralPath (Join-Path $bin 'docker.exe')) { $env:Path = "$bin;$env:Path"; break }
        }
    }
    if (-not (Get-Command docker -ErrorAction SilentlyContinue)) {
        Say 'Docker Desktop is not installed.'
        Say 'Install it from https://docs.docker.com/desktop/setup/install/windows-install/'
        Say '(keep "Use WSL 2" ticked), restart Windows if asked, then run this again.'
        exit 1
    }
    $info = Docker-Quiet info
    if ($LASTEXITCODE -ne 0) {
        $text = ($info | ForEach-Object { "$_" }) -join "`n"
        if ($text -match 'Access is denied|permission denied') {
            Fail 'you are not allowed to use Docker yet. Add yourself to the docker-users group (as an administrator: net localgroup docker-users <your user name> /add), then sign out and back in.'
        }
        $exe = $desktopDirs | ForEach-Object { Join-Path $_ 'Docker Desktop.exe' } |
            Where-Object { Test-Path -LiteralPath $_ } | Select-Object -First 1
        if ($exe) {
            Step 'Starting Docker Desktop'
            Start-Process $exe | Out-Null
        } else {
            Fail 'Docker is not running. Start Docker Desktop and try again.'
        }
        $ready = $false
        for ($i = 0; $i -lt 90; $i++) {
            Start-Sleep -Seconds 2
            Docker-Quiet info | Out-Null
            if ($LASTEXITCODE -eq 0) { $ready = $true; break }
        }
        if (-not $ready) { Fail 'Docker Desktop did not start within 3 minutes. Start it by hand and try again.' }
    }
    $osType = First-Text (Docker-Quiet info --format '{{.OSType}}')
    if ($osType -and $osType.Trim() -ne 'linux') {
        Fail 'Docker Desktop is set to Windows containers. Right-click the Docker icon in the taskbar and choose "Switch to Linux containers...", then try again.'
    }
}

function Container-State {
    $state = Docker-Quiet container inspect -f '{{.State.Status}}' $Name
    $text = First-Text $state
    if ($LASTEXITCODE -ne 0 -or -not $text) { return 'none' }
    return $text.Trim()
}

function Check-Resources {
    try {
        $cpus = 0; $mem = 0.0
        [int]::TryParse((First-Text (Docker-Quiet info --format '{{.NCPU}}')), [ref]$cpus) | Out-Null
        [double]::TryParse((First-Text (Docker-Quiet info --format '{{.MemTotal}}')), [ref]$mem) | Out-Null
        $memGb = [math]::Floor($mem / 1GB)
        if ($cpus -gt 0 -and $cpus -lt 4) {
            Warn "Docker can use only $cpus CPU(s); the simulation runs better with 4 or more."
        }
        if ($memGb -gt 0 -and $memGb -lt 4) {
            Warn "Docker has only about $memGb GB of memory; 4 GB or more is recommended (WSL settings)."
        }
    } catch { }
}

function Pull-Image {
    Step "Downloading FIC Studio ($Image); the first time takes a few minutes"
    # Shown as it happens (a first download is about 1 GB) and kept to read
    # the error from.
    $previous = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    try {
        & docker pull $Image 2>&1 | Tee-Object -Variable out | ForEach-Object {
            if ($_ -is [System.Management.Automation.ErrorRecord]) { Write-Host $_.Exception.Message } else { Write-Host "$_" }
        }
        $code = $LASTEXITCODE
    } finally { $ErrorActionPreference = $previous }
    if ($code -ne 0) {
        $text = ($out | ForEach-Object { "$_" }) -join "`n"
        if ($text -match 'manifest unknown|not found|does not exist') {
            Fail "$Image does not exist (yet). Check the name and tag, or use -Image."
        } elseif ($text -match 'denied|unauthorized') {
            Fail "not allowed to download $Image. If it is private, log in first:  docker login $($Image.Split('/')[0])"
        }
        Fail "could not download $Image (see the message above)."
    }
}

function Ensure-Image {
    Docker-Quiet image inspect $Image | Out-Null
    if ($LASTEXITCODE -ne 0) { Pull-Image }
}

$script:Password = ''
function Browser-Url {
    $url = "http://localhost:$Port/vnc.html"
    if ($script:Password) { $url += "?password=$($script:Password)" }
    return $url
}

function New-Password {
    $chars = [char[]]'ABCDEFGHJKLMNPQRSTUVWXYZabcdefghijkmnpqrstuvwxyz23456789'
    $bytes = New-Object byte[] 8
    [System.Security.Cryptography.RandomNumberGenerator]::Create().GetBytes($bytes)
    return -join ($bytes | ForEach-Object { $chars[$_ % $chars.Length] })
}

function Container-Env([string]$Key) {
    foreach ($line in (Docker-Quiet container inspect -f '{{range .Config.Env}}{{println .}}{{end}}' $Name)) {
        if ("$line" -like "$Key=*") { return "$line".Substring($Key.Length + 1) }
    }
    return ''
}

function Open-Url([string]$Url) {
    if (-not $NoBrowser) { Start-Process $Url | Out-Null }
}

function Lan-Address {
    try {
        return (Find-NetRoute -RemoteIPAddress '1.1.1.1' -ErrorAction Stop |
            Where-Object { $_.IPAddress } | Select-Object -First 1).IPAddress
    } catch { return '' }
}

function Wait-ForDesktop {
    # The port is published on 127.0.0.1 only; 'localhost' may try ::1 first.
    $url = "http://127.0.0.1:$Port/vnc.html"
    Write-Host -NoNewline 'Waiting for the desktop'
    for ($i = 0; $i -lt 120; $i++) {
        if ((Container-State) -ne 'running') {
            Write-Host ''
            Docker-Show logs --tail 40 $Name
            Fail 'FIC Studio stopped while starting (see the messages above).'
        }
        try {
            Invoke-WebRequest -Uri $url -UseBasicParsing -TimeoutSec 5 | Out-Null
            Write-Host ' ready.'
            return
        } catch {
            Write-Host -NoNewline '.'
            Start-Sleep -Seconds 1
        }
    }
    Write-Host ''
    Warn "the desktop is taking longer than usual; try $(Browser-Url) in a moment."
}

# Where phones open the web remote, and the sign-in they need (-Lan only).
function Lan-Message([string]$AdminPassword) {
    if (-not $AdminPassword) { return }
    $ip = Lan-Address
    if (-not $ip) { $ip = '<this computer>' }
    Say "Phones on your network: http://${ip}:${WebPort}/  (sign in as admin, password $AdminPassword)"
}

function Start-Studio {
    Ensure-Docker
    $state = Container-State
    if ($state -eq 'running') {
        $mapped = First-Text (Docker-Quiet port $Name 6080/tcp)
        if ($mapped -match ':(\d+)\s*$') { $script:Port = [int]$Matches[1] }
        $script:Password = Container-Env 'FIC_VNC_PASSWORD'
        Say 'FIC Studio is already running.'
        Say "Open $(Browser-Url)"
        Open-Url (Browser-Url)
        $mappedWeb = First-Text (Docker-Quiet port $Name 8088/tcp)
        if ($mappedWeb -match ':(\d+)\s*$') { $script:WebPort = [int]$Matches[1] }
        Lan-Message (Container-Env 'FIC_WEB_ADMIN_PASSWORD')
        Say 'To start it with different options:  .\fic-studio.ps1 restart [options]'
        return
    }

    Ensure-Image
    Check-Resources

    if ($state -ne 'none') { Docker-Quiet rm -f $Name | Out-Null }

    $folderPath = (New-Item -ItemType Directory -Force -Path $Folder).FullName
    $script:Password = New-Password
    $env:FIC_VNC_PASSWORD = $script:Password
    $env:FIC_URL = Browser-Url

    $runArgs = @(
        'run', '--name', $Name, '--hostname', 'fic-studio', '--detach',
        '--shm-size', '2g',
        '-v', "${Volume}:/home/fic",
        '-v', "${folderPath}:/home/fic/FIC_Studio",
        # The password, and the link carrying it, go through the environment
        # (-e NAME): a command line is visible to other users of this computer.
        '-e', 'FIC_URL',
        '-e', 'FIC_VNC_PASSWORD',
        '-e', "FIC_WEB_REMOTE_PORT=$WebPort",
        # Only this computer can reach the desktop, and only with the password
        # of this start.
        '-p', "127.0.0.1:${Port}:6080"
    )
    if ($Lan) {
        $ip = Lan-Address
        $hosts = if ($ip) { "$ip,localhost" } else { 'localhost' }
        $runArgs += @('-p', "${WebPort}:8088", '-e', "FIC_WEB_REMOTE_HOSTS=$hosts")
        # Open to the network, so sign-in is on: the account admin gets a new
        # password every start (through the environment, like the desktop's).
        $adminPassword = New-Password
        $env:FIC_WEB_ADMIN_PASSWORD = $adminPassword
        $runArgs += @('-e', 'FIC_WEB_ADMIN_PASSWORD')
    } else {
        $runArgs += @('-p', "127.0.0.1:${WebPort}:8088")
    }
    $mode = if ($Dev) { 'dev' } elseif ($Desktop) { 'desktop' } else { 'studio' }
    $runArgs += @($Image, $mode)

    Step 'Starting FIC Studio'
    try {
        $out = Docker-Quiet @runArgs
        $code = $LASTEXITCODE
    } finally {
        # Only docker needed them; do not leave them in this PowerShell session.
        Remove-Item Env:FIC_VNC_PASSWORD, Env:FIC_URL, Env:FIC_WEB_ADMIN_PASSWORD -ErrorAction SilentlyContinue
    }
    if ($code -ne 0) {
        Show-Output $out
        Docker-Quiet rm -f $Name | Out-Null
        Fail 'Docker could not start FIC Studio (see the message above). If a port is in use, choose another with -Port N (browser) or -WebPort N (web remote).'
    }

    Wait-ForDesktop
    Say ''
    Say "FIC Studio is running:  $(Browser-Url)"
    Say "(The link includes this session's password; .\fic-studio.ps1 shows it again.)"
    Open-Url (Browser-Url)
    if ($Lan) { Lan-Message $adminPassword }
    Say "Your shared folder:     $folderPath"
    Say 'Stop it with:           .\fic-studio.ps1 stop'
}

function Confirm([string]$Question) {
    $reply = Read-Host "$Question [y/N]"
    return $reply -match '^[Yy]'
}

switch ($Command) {
    'help' { Get-Help $PSCommandPath -Detailed; break }
    'start' { Start-Studio; break }
    'stop' {
        Ensure-Docker
        if ((Container-State) -eq 'running') {
            Step 'Stopping FIC Studio (your settings are saved)'
            Docker-Quiet stop -t 20 $Name | Out-Null
        } else { Say 'FIC Studio is not running.' }
        break
    }
    'restart' {
        Ensure-Docker
        # Make sure the image can be used before stopping the running one.
        Ensure-Image
        if ((Container-State) -ne 'none') { Docker-Quiet stop -t 20 $Name | Out-Null }
        Start-Studio
        break
    }
    'update' {
        Ensure-Docker
        Pull-Image
        if ((Container-State) -ne 'none') {
            Step 'Replacing the running version (your settings are kept)'
            Docker-Quiet stop -t 20 $Name | Out-Null
            Docker-Quiet rm $Name | Out-Null
        }
        Start-Studio
        break
    }
    'logs' {
        Ensure-Docker
        if ((Container-State) -eq 'none') { Fail 'FIC Studio has not been started yet (start it with .\fic-studio.ps1).' }
        Docker-Show logs -f --tail 200 $Name
        break
    }
    'shell' {
        Ensure-Docker
        if ((Container-State) -ne 'running') { Fail 'FIC Studio is not running (start it with .\fic-studio.ps1).' }
        if ([Console]::IsInputRedirected -or [Console]::IsOutputRedirected) {
            Docker-Show exec -i $Name gosu fic fic shell
        } else {
            Docker-Show exec -it $Name gosu fic fic shell
        }
        if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
        break
    }
    'status' {
        Ensure-Docker
        $state = Container-State
        Say "Container: $state"
        if ($state -ne 'none') {
            Say "Image:     $(First-Text (Docker-Quiet container inspect -f '{{.Config.Image}}' $Name))"
            $imageId = First-Text (Docker-Quiet container inspect -f '{{.Image}}' $Name)
        } else {
            $imageId = $Image
        }
        $created = First-Text (Docker-Quiet image inspect -f '{{.Created}}' $imageId)
        if ($LASTEXITCODE -eq 0) { Say "Created:   $created" } else { Say "Image:     $Image (not downloaded yet)" }
        if ($state -eq 'running') { Docker-Show exec $Name gosu fic fic-state status }
        break
    }
    'reset' {
        Ensure-Docker
        if (-not (Confirm "Forget all FIC Studio settings, tuning and robots added in the Studio? (The shared folder $Folder is kept.)")) {
            Say 'Nothing changed.'; break
        }
        Docker-Quiet rm -f $Name | Out-Null
        Docker-Quiet volume inspect $Volume | Out-Null
        if ($LASTEXITCODE -eq 0) {
            $out = Docker-Quiet volume rm $Volume
            if ($LASTEXITCODE -ne 0) { Show-Output $out; Fail "could not remove the settings volume $Volume." }
        }
        Say 'Done. Start again with .\fic-studio.ps1'
        break
    }
    'uninstall' {
        Ensure-Docker
        if (-not (Confirm "Remove the FIC Studio container, its settings volume and the image? (The shared folder $Folder is kept.)")) {
            Say 'Nothing changed.'; break
        }
        Docker-Quiet rm -f $Name | Out-Null
        Docker-Quiet volume inspect $Volume | Out-Null
        if ($LASTEXITCODE -eq 0) {
            Docker-Quiet volume rm $Volume | Out-Null
            if ($LASTEXITCODE -eq 0) { Say "Removed the settings volume $Volume." } else { Warn "could not remove the settings volume $Volume." }
        }
        Docker-Quiet image inspect $Image | Out-Null
        if ($LASTEXITCODE -eq 0) {
            Docker-Quiet image rm $Image | Out-Null
            if ($LASTEXITCODE -eq 0) { Say "Removed the image $Image." } else { Warn "could not remove the image $Image (is another container using it?)." }
        }
        Say 'FIC Studio has been removed from Docker.'
        break
    }
}
