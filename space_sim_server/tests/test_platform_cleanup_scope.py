"""Stopping one workspace must not match another workspace's identical module."""
import os
from pathlib import Path
import shutil
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[1]
PWSH = shutil.which('pwsh')


@pytest.mark.skipif(os.name != 'nt' or not PWSH, reason='Windows PowerShell required')
def test_orphan_cleanup_is_scoped_to_private_interpreter(tmp_path):
    harness = tmp_path / 'scope.ps1'
    harness.write_text(r'''
param($Source)
$ErrorActionPreference='Stop'
$tokens=$null; $errors=$null
$ast=[Management.Automation.Language.Parser]::ParseFile($Source,[ref]$tokens,[ref]$errors)
if($errors.Count){throw 'Invalid PowerShell'}
$fn=$ast.Find({param($n) $n -is [Management.Automation.Language.FunctionDefinitionAst] -and $n.Name -eq 'Stop-OrphanedPlatformBackends'},$true)
Invoke-Expression $fn.Extent.Text
$projectRoot='C:\work\mine\space_sim_server'; $workspaceRoot='C:\work\mine'; $Quiet=$true
function Get-CimInstance {
    @(
        [pscustomobject]@{ProcessId=101;CommandLine='"C:\work\mine\.venv\Scripts\python.exe" -m space_arm_platform.main'},
        [pscustomobject]@{ProcessId=102;CommandLine='"C:\work\other\.venv\Scripts\python.exe" -m space_arm_platform.main'},
        [pscustomobject]@{ProcessId=103;CommandLine='"C:\work\mine-other\.venv\Scripts\python.exe" -m space_arm_platform.main'},
        [pscustomobject]@{ProcessId=104;CommandLine='"C:/work/mine/.python/cpython/python.exe" -m space_arm_platform.main'},
        [pscustomobject]@{ProcessId=105;CommandLine='python.exe -m space_arm_platform.main'}
    )
}
function Get-Process {param($Id,$ErrorAction) [pscustomobject]@{StartTime=[DateTime]::new(1000,[DateTimeKind]::Utc)}}
function Stop-RecordedProcessTree {param($ProcessId,$ExpectedStartTicks)
    if($ExpectedStartTicks -ne 1000){throw 'Missing process identity'}
    "STOP-$ProcessId"
}
Stop-OrphanedPlatformBackends
''', encoding='utf-8')
    result = subprocess.run([PWSH, '-NoProfile', '-File', str(harness),
                             str(ROOT / 'scripts/stop_platform.ps1')],
                            capture_output=True, text=True, timeout=20)
    assert result.returncode == 0, result.stdout + result.stderr
    assert result.stdout.split() == ['STOP-101', 'STOP-104']
