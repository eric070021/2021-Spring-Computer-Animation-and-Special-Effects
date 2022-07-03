if (!(Test-Path -Path "src")) {
  cd ..
}

if (!(Test-Path -Path "src")) {
  Write-Output "Please run this script in your homework folder."
  exit 1
}

if ($args.count -eq 1) {
  $studentid = [string]$args[0]
  Write-Output "Packing for $studentid"
  $compress = @{
    Path = "include", "src"
    CompressionLevel = "Optimal"
    DestinationPath = "hw1_$studentid.zip"
  }
  Compress-Archive @compress
} else {
  Write-Output "Usage: $PSCommandPath <student_id>"
}