# word_pdf.ps1 DOC.docx OUT.pdf [TIMEOUT_S]
# Export a .docx to PDF with Word, read-only, with a hard timeout.
# Word is started in its own process; if it does not finish in time, only that
# process is stopped, so a hidden dialog can never leave Word hanging.
param([string]$doc, [string]$out, [int]$timeout = 180)
$before = @(Get-Process WINWORD -ErrorAction SilentlyContinue | ForEach-Object Id)
$job = Start-Job -ArgumentList $doc, $out -ScriptBlock {
  param($doc, $out)
  $w = New-Object -ComObject Word.Application
  $w.Visible = $false; $w.DisplayAlerts = 0
  try {
    $d = $w.Documents.Open($doc, $false, $true)    # no conversion dialog, read-only
    $d.ExportAsFixedFormat($out, 17)               # 17 = wdExportFormatPDF
    $d.Close(0)
  } finally { $w.Quit() }
}
if (Wait-Job $job -Timeout $timeout) {
  Receive-Job $job
  Write-Output "ok: $out"
} else {
  Stop-Job $job
  Get-Process WINWORD -ErrorAction SilentlyContinue |
    Where-Object { $before -notcontains $_.Id } |
    Stop-Process -Force -Confirm:$false
  Write-Output "timeout after $timeout s: Word stopped, no PDF"
  exit 1
}
Remove-Job $job -Force
