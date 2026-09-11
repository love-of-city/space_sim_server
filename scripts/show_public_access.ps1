param([string]$ConfigPath = '', [switch]$ValidateOnly)
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$explicitConfig = [bool]$ConfigPath
if (!$ConfigPath) { $ConfigPath = Join-Path $root 'deploy/public.local.json' }
$ConfigPath = [IO.Path]::GetFullPath($ConfigPath, $root)
Add-Type -AssemblyName System.Windows.Forms
Add-Type -AssemblyName System.Drawing
try {
    $report = Get-Content -Raw -LiteralPath (Join-Path $root 'run/public-access.json') | ConvertFrom-Json
    # The launcher may have used the fixed-public config instead of the
    # temporary-tunnel config; follow the report unless the caller was explicit.
    if (!$explicitConfig -and $report.config_path) {
        $ConfigPath = [IO.Path]::GetFullPath([string]$report.config_path, $root)
    }
    if ($report.config_path -ne $ConfigPath) { throw 'The access report belongs to another configuration.' }
    $hash = [Security.Cryptography.SHA256]::Create()
    try { $id = [BitConverter]::ToString($hash.ComputeHash([Text.Encoding]::UTF8.GetBytes($ConfigPath.ToLowerInvariant()))).Replace('-', '') }
    finally { $hash.Dispose() }
    $saved = Import-Clixml -LiteralPath (Join-Path $root "deploy/secrets/$id.clixml")
    $access = [pscredential]::new('deployment',$saved.SPACE_SIM_STREAM_ACCESS_KEY).GetNetworkCredential().Password
    # Older reports predate this field; they were all access-key protected.
    $requireKey = $true
    if ($report.PSObject.Properties['require_access_key']) { $requireKey = [bool]$report.require_access_key }
    $password = [pscredential]::new('deployment',$saved.SPACE_SIM_ADMIN_PASSWORD).GetNetworkCredential().Password
    $form = [Windows.Forms.Form]::new()
    $reportMode = [string]$report.mode
    $isIp = $reportMode -eq 'ip'
    $isFixed = @('fixed', 'ip') -contains $reportMode
    $form.Text = if ($isIp) { '仿真平台 · 固定公网 IP 访问' } elseif ($isFixed) { '仿真平台 · 固定公网访问' } else { '仿真平台 · 临时公网访问' }
    $form.Size = [Drawing.Size]::new(790,450)
    $form.StartPosition = 'CenterScreen'
    $form.Font = [Drawing.Font]::new('Microsoft YaHei UI',10)
    function Add-AccessLabel([string]$Text,[int]$Y) {
        $label=[Windows.Forms.Label]::new(); $label.Text=$Text; $label.SetBounds(20,$Y,735,48); $form.Controls.Add($label)
    }
    function Add-AccessField([string]$Text,[int]$Y,[bool]$Secret=$false) {
        $box=[Windows.Forms.TextBox]::new(); $box.Text=$Text; $box.ReadOnly=$true; $box.UseSystemPasswordChar=$Secret
        $box.SetBounds(20,$Y,735,30); $form.Controls.Add($box); return $box
    }
    if ($requireKey) {
        Add-AccessLabel '在用户自己的浏览器打开下方链接，手柄也接在用户电脑上。链接含访问密钥，请仅交给获授权的用户。' 15
    } else {
        Add-AccessLabel '在用户自己的浏览器打开下方链接，手柄也接在用户电脑上。本次已关闭访问密钥，请只把网址交给可信用户。' 15
    }
    $linkUrl = if ($requireKey) { $report.public_url+'/?access_key='+$access } else { $report.public_url+'/' }
    $linkBox = Add-AccessField $linkUrl 65
    $copy = [Windows.Forms.Button]::new(); $copy.Text='复制访问链接'; $copy.SetBounds(20,105,170,32)
    $copy.Add_Click({try {[Windows.Forms.Clipboard]::SetText($linkBox.Text)} catch {[Windows.Forms.MessageBox]::Show('剪贴板不可用，请选择文本手动复制。') | Out-Null}})
    $form.Controls.Add($copy)
    $generated = @($report.generated_password_users)
    if ($generated.Count -gt 0) {
        Add-AccessLabel ('自动初始化/默认密码已更换的管理员：'+($generated -join ', ')+'。若之后在网页改过密码，请用修改后的密码。') 150
        $passwordBox = Add-AccessField $password 200 $true
        $reveal=[Windows.Forms.CheckBox]::new(); $reveal.Text='显示初始化密码'; $reveal.SetBounds(20,237,220,28)
        $reveal.Add_CheckedChanged({$passwordBox.UseSystemPasswordChar=!$reveal.Checked}); $form.Controls.Add($reveal)
    } else { Add-AccessLabel ('已有管理员：'+(@($report.existing_admin_users) -join ', ')+'；自定义密码没有更改，请使用原密码。') 150 }
    if (!$requireKey) {
        Add-AccessLabel '注意：访问密钥已关闭。任何人只要知道本网址即可打开登录页，安全仅依赖账号密码。' 335
    }
    if ($isIp) {
        Add-AccessLabel '此地址为固定公网 IP 入口，使用 Let''s Encrypt IP 证书（约 6 天，自动续期）。证书续期依赖本机 Caddy 持续运行，长时间关机后请重新启动部署。' 275
    } elseif ($isFixed) {
        Add-AccessLabel '此地址为固定公网入口，使用本机 Caddy 自动签发 HTTPS，并已接入本机 TURN。请按所在单位规定处理仿真数据。' 275
        Add-AccessLabel '网页入口已验证。视频仍建议在用户自己的网络和浏览器中实际验收。关闭此窗口不停止平台；使用 stop_deployment.cmd 停止。' 335
    } else {
        Add-AccessLabel '此网址由第三方临时隧道提供，重启可能变化，无生产可用性保证。HTTPS 在第三方边缘终止；不要上传敏感仿真数据。' 275
        Add-AccessLabel '网页入口已验证。视频需要另外验证 WebRTC，部分网络需 TURN。关闭此窗口不停止平台；使用 stop_deployment.cmd 停止。' 335
    }
    if ($ValidateOnly) {
        Write-Output 'Encrypted credentials and access-window controls validated; no window, clipboard, or secret output.'
    } else {
        # The launcher hides the helper's console via STARTUPINFO. Consume that
        # first-show override with an explicit hide; otherwise Windows can also
        # hide the first WinForms dialog even while ShowDialog is running.
        Add-Type @"
using System;
using System.Runtime.InteropServices;
public static class SpaceSimAccessWindow {
    [DllImport("user32.dll")]
    public static extern bool ShowWindow(IntPtr handle, int command);
}
"@
        $null=[SpaceSimAccessWindow]::ShowWindow($form.Handle, 0)
        $null=$form.ShowDialog()
    }
} catch {
    if ($ValidateOnly) { throw 'Access-window validation failed. No credentials were displayed.' }
    $null=[Windows.Forms.MessageBox]::Show('无法读取当前公网部署的访问资料。请先运行 start_deployment.cmd，并使用同一 Windows 账号。不会在此输出密钥。','仿真平台')
    exit 1
}
finally {
    if ($form) { $form.Dispose() }
}


