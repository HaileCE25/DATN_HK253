<#
.SYNOPSIS
  Thống kê sai số khoảng cách UWB từ file log của PlatformIO (logs/device-monitor-*.log).

.DESCRIPTION
  Đọc các dòng "[UWB DW3000] <ms> ms distance=<d> m (raw=<r>)", so với khoảng cách thật đo bằng thước.
  Hai cách dùng:

  1) Một khoảng cách, đứng yên cả phiên:
       .\tools\uwb_stats.ps1 -Truth 2.38
       .\tools\uwb_stats.ps1 -Truth 2.38 -SkipSec 60        # bỏ 60 s đầu (chip chưa ổn định)

  2) Nhiều khoảng cách trong cùng một phiên (hiệu chuẩn): mỗi đoạn "khoảng_cách_thật:từ_giây:đến_giây"
     (giây theo mốc thời gian trong log, tức số ms/1000):
       .\tools\uwb_stats.ps1 -Segments "0.5:20:50","1.0:60:90","1.5:100:130","2.0:140:170","3.0:180:210"

  Nếu có từ 2 đoạn trở lên, in thêm đường thẳng khớp  thật = scale * đo + offset  (bình phương tối thiểu),
  dùng làm cơ sở cho UWB_RANGE_SCALE / UWB_RANGE_OFFSET_CM.

  Mặc định lấy file log mới nhất trong logs/; chỉ định file khác bằng -Log.
#>
param(
    [string]   $Log,
    [switch]   $Raw,   # dùng file dữ liệu thô đầy đủ logs/uwb_raw-*.csv mới nhất (mọi round)
    [switch]   $ByBoot, # tách theo lần khởi động lại (mốc ms tụt về nhỏ) và so sánh các lần
    [double]   $Truth = [double]::NaN,
    [double]   $SkipSec = 0,
    [string[]] $Segments
)

$root = Split-Path -Parent $PSScriptRoot
if (-not $Log) {
    $pattern = if ($Raw) { 'uwb_raw-*.csv' } else { '*.log' }
    $f = Get-ChildItem (Join-Path $root 'logs') -Filter $pattern | Sort-Object LastWriteTime | Select-Object -Last 1
    if (-not $f) { Write-Error "Khong thay file log trong logs/"; exit 1 }
    $Log = $f.FullName
}
Write-Host "Log: $Log"

$rows = foreach ($l in (Get-Content $Log)) {
    if ($l -match '(\d+) ms distance=([\d.]+) m \(raw=([\d.]+)\)') {
        [pscustomobject]@{ t = [int]$matches[1] / 1000.0; d = [double]$matches[2]; r = [double]$matches[3] }
    }
    elseif ($l -match '^\s*(\d+),([\d.]+),([\d.]+),([\d.]+),([\d.]+),(\d)\s*$') {
        # CSV luồng thô mới: ms,raw_cm,med_cm,filt_cm,alpha,flag
        [pscustomobject]@{ t = [int]$matches[1] / 1000.0; r = [double]$matches[2] / 100.0; d = [double]$matches[4] / 100.0 }
    }
    elseif ($l -match '^\s*(\d+),([\d.]+),([\d.]+)\s*$') {
        # CSV luồng thô cũ: ms,raw_cm,filt_cm (bỏ qua dòng FAIL vì cột 2 không phải số)
        [pscustomobject]@{ t = [int]$matches[1] / 1000.0; r = [double]$matches[2] / 100.0; d = [double]$matches[3] / 100.0 }
    }
}
if (-not $rows) { Write-Error "Khong co dong 'distance=... (raw=...)' nao trong log"; exit 1 }

function Get-Stats([double[]]$v) {
    $n = $v.Count
    $m = ($v | Measure-Object -Average).Average
    $s = 0.0
    if ($n -gt 1) { $s = [math]::Sqrt((($v | ForEach-Object { ($_ - $m) * ($_ - $m) }) | Measure-Object -Sum).Sum / ($n - 1)) }
    [pscustomobject]@{ N = $n; Mean = $m; Std = $s
                       Min = ($v | Measure-Object -Minimum).Minimum; Max = ($v | Measure-Object -Maximum).Maximum }
}

function Show-Segment($name, $seg, $truth) {
    if ($seg.Count -lt 2) { Write-Host "$name : khong du mau ($($seg.Count))"; return $null }
    $r = Get-Stats $seg.r
    $d = Get-Stats $seg.d
    Write-Host ("{0}: n={1}  raw mean={2:N3} std={3:N3} [{4:N2}..{5:N2}]  filtered mean={6:N3} std={7:N3}" -f $name, $r.N, $r.Mean, $r.Std, $r.Min, $r.Max, $d.Mean, $d.Std)
    if (-not [double]::IsNaN($truth)) {
        Write-Host ("      thuoc={0:N3} m   lech(raw)={1:+0.000;-0.000} m   lech(filtered)={2:+0.000;-0.000} m" -f $truth, ($r.Mean - $truth), ($d.Mean - $truth))
    }
    return [pscustomobject]@{ Truth = $truth; RawMean = $r.Mean }
}

if ($ByBoot) {
    # Mỗi lần Car khởi động lại, mốc ms bắt đầu từ đầu -> phát hiện khi t tụt xuống.
    $rows = @($rows)
    $boot = 0; $prev = -1.0
    foreach ($r0 in $rows) {
        if ($r0.t -lt $prev) { $boot++ }
        $prev = $r0.t
        $r0 | Add-Member -NotePropertyName b -NotePropertyValue $boot
    }
    Write-Host "Theo tung lan khoi dong (bo $SkipSec s dau moi lan):"
    $means = @()
    foreach ($g in ($rows | Group-Object b)) {
        $seg = @($g.Group | Where-Object { $_.t -ge $SkipSec })
        if ($seg.Count -lt 2) { continue }
        $sd = Get-Stats $seg.d
        $sr = Get-Stats $seg.r
        $means += $sd.Mean
        $line = "  lan {0}: n={1,5}  filtered mean={2:N3} std={3:N3}  raw std={4:N3}" -f ([int]$g.Name + 1), $seg.Count, $sd.Mean, $sd.Std, $sr.Std
        if (-not [double]::IsNaN($Truth)) { $line += ("  lech={0:+0.000;-0.000} m" -f ($sd.Mean - $Truth)) }
        Write-Host $line
    }
    if ($means.Count -ge 2) {
        $lo = ($means | Measure-Object -Minimum).Minimum; $hi = ($means | Measure-Object -Maximum).Maximum
        Write-Host ("`nLech giua cac lan khoi dong: {0:N1} cm  (max - min cua trung binh; nho la thiet bi lap lai tot)" -f (($hi - $lo) * 100))
    }
    exit 0
}

if ($Segments) {
    # Chấp nhận cả "a","b" (PowerShell) lẫn "a,b" (khi gọi qua -File, các phần tử bị gộp thành 1 chuỗi).
    $Segments = @($Segments | ForEach-Object { $_ -split ',' } | Where-Object { $_.Trim() })
    $points = @()
    foreach ($s in $Segments) {
        $p = $s -split ':'
        if ($p.Count -ne 3) { Write-Error "Doan sai dinh dang '$s' (can 'that:tu:den')"; exit 1 }
        $tr = [double]$p[0]; $a = [double]$p[1]; $b = [double]$p[2]
        $seg = @($rows | Where-Object { $_.t -ge $a -and $_.t -le $b })
        $pt = Show-Segment ("{0:N2} m [{1}-{2}s]" -f $tr, $a, $b) $seg $tr
        if ($pt) { $points += $pt }
    }
    if ($points.Count -ge 2) {
        $n = $points.Count
        $sx = ($points.RawMean | Measure-Object -Sum).Sum
        $sy = ($points.Truth | Measure-Object -Sum).Sum
        $sxx = ($points | ForEach-Object { $_.RawMean * $_.RawMean } | Measure-Object -Sum).Sum
        $sxy = ($points | ForEach-Object { $_.RawMean * $_.Truth } | Measure-Object -Sum).Sum
        $den = $n * $sxx - $sx * $sx
        if ([math]::Abs($den) -gt 1e-9) {
            $scale = ($n * $sxy - $sx * $sy) / $den
            $offset = ($sy - $scale * $sx) / $n
            Write-Host ""
            Write-Host ("Khop  that = scale * do + offset:  scale={0:N4}  offset={1:+0.000;-0.000} m ({2:+0.0;-0.0} cm)" -f $scale, $offset, ($offset * 100))
            Write-Host "Sai so con lai sau khi bu:"
            foreach ($pt in $points) {
                $fit = $scale * $pt.RawMean + $offset
                Write-Host ("   thuoc {0:N2} m -> {1:N3} m   (sai {2:+0.0;-0.0} cm)" -f $pt.Truth, $fit, (($fit - $pt.Truth) * 100))
            }
        }
    }
}
else {
    $sel = @($rows | Where-Object { $_.t -ge $SkipSec })
    [void](Show-Segment "Toan phien (bo $SkipSec s dau)" $sel $Truth)

    Write-Host ""
    Write-Host "Theo cua so 10 s (loc, trung binh):"
    $sel | Group-Object { [int][math]::Floor($_.t / 10) } | ForEach-Object {
        $a = ($_.Group.d | Measure-Object -Average).Average
        $line = "  {0,4}-{1,4} s  n={2,3}  filtered={3:N3}" -f ([int]$_.Name * 10), ([int]$_.Name * 10 + 10), $_.Count, $a
        if (-not [double]::IsNaN($Truth)) { $line += ("  lech={0:+0.000;-0.000}" -f ($a - $Truth)) }
        Write-Host $line
    }
}
