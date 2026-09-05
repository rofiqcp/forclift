# YOLOPv2 model untuk Mini-PC CPU

Lokasi default model yang dipakai source ini:

    /home/sirobo/ros/models/yolopv2.pt

Bobot resmi YOLOPv2 V0.0.1 berukuran 156380200 byte dengan SHA-256:

    f2a8c8374203ae3e67ff9c184e931f763957de92a993b23269e4e721627f1f8c

Jika file `yolopv2.pt` belum ada, jalankan dari root workspace:

    python3 src/perception/tools/download_yolopv2.py

Downloader memakai release resmi CAIC-AD dan memverifikasi ukuran + SHA-256 sebelum file dipasang.
