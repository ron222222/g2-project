Get-ChildItem .\images\train\*.jpg | ForEach-Object {

    $txt = ".\\labels\\train\\" + $_.BaseName + ".txt"

    if (!(Test-Path $txt)) {

        New-Item -ItemType File -Path $txt | Out-Null

    }

}