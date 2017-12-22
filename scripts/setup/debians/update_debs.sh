#! /bin/bash

# Run this script from the directory with a debian package repository.
# It will sign all the packages in the repository, create all the helper
# files, and dupload everything to volar.

# The debian repository must have beforehand:
#    dists/xenial/main/binary-armhf with unsigned armhf .deb files
#    dists/xenial/main/binary-amd64 with unsigned amd64 .deb files
#    dists/xenial/main/source with .orig.tar.gz, .debian.tar.gz, and .dsc files
# Given these files, the script will create everything else.
# You must also have the astrobee private key on your computer.

targetdir=dists/xenial/main/binary-armhf
mkdir -p $targetdir
for f in $targetdir/*.deb
do
  dpkg-sig -k 6E011D76 --sign builder $f
done
dpkg-scanpackages $targetdir /dev/null > $targetdir/Packages
gzip -9c $targetdir/Packages > $targetdir/Packages.gz

targetdir=dists/xenial/main/binary-amd64
mkdir -p $targetdir
for f in $targetdir/*.deb
do
  dpkg-sig -k 6E011D76 --sign builder $f
done
dpkg-scanpackages $targetdir /dev/null > $targetdir/Packages
gzip -9c $targetdir/Packages > $targetdir/Packages.gz

targetdir=dists/xenial/main/source
#for f in $targetdir/*.deb
#do
#  dpkg-sig -k 6E011D76 --sign builder $f
#done
mkdir -p $targetdir
dpkg-scansources $targetdir /dev/null > $targetdir/Sources
gzip -9c $targetdir/Sources > $targetdir/Sources.gz

rm -f dists/xenial/Release
rm -f dists/xenial/Release.gpg
rm -f dists/xenial/InRelease
apt-ftparchive release dists/xenial > Release
mv Release dists/xenial
cd dists/xenial
gpg --default-key 6E011D76 --clearsign -o InRelease Release 
gpg --default-key 6E011D76 -abs -o Release.gpg Release

cd ../..
chmod -R g+r dists/
chmod -R o+r dists/
rsync -avz --delete dists volar:/home/p-free-flyer/free-flyer/FSW/ars_debs/
