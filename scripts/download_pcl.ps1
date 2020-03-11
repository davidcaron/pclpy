$pclversion = "1.9.1"

Invoke-WebRequest -OutFile "pcl-$pclversion.zip" "https://github.com/PointCloudLibrary/pcl/archive/pcl-$pclversion.zip"
Expand-Archive -Force "pcl-$pclversion.zip" "pcl-pcl-$pclversion"
Remove-Item "pcl-$pclversion.zip"
