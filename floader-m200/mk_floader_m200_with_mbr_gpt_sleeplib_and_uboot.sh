cp -rf ~/workspace/for-5/floader-m200/floader_m200_with_mbr_gpt_and_sleeplib.bin .
cat floader_m200_with_mbr_gpt_and_sleeplib.bin u-boot.bin > u-boot-with-spl-mbr-gpt.bin
cp -rf u-boot-with-spl-mbr-gpt.bin ~/
