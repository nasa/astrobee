# Debians management for deployment on the physical robots

This page describes the steps to deploy FSW Debian packages on the server.

The Debian are deploy in 3 phases:
  1. Build the Debian package (freeflyer or freeflyer_avionics)
  2. Stage the package on volar
  3. Create a new Debian snapshot on the astrobee server

Steps 2 and 3 are similar for any Debian update
Step 1 is slightly different for freeflyer or freeflyer_avionics.

Setup:

    export FF_SOURCE=<path_to_your_freeflyer_repository>
    export FF_AVIONICS=<path_to_your_avionics_repository>
        
More information on how Astrobee is using
[aptly](https://www.aptly.info/doc/overview/) is described in:
[Debians mirror documentation](../scripts/setup/Debians/mirror/README.md)
          

## Build Freeflyer Debian

Your environment needs to be configured properly to cross-compile Astrobee
Robot Software for the `armhf` architecture:
[see NASA Install documentation](../NASA_INSTALL.md)

### Update the release version

    cd $FF_SOURCE
    ./scripts/setup/debians/update_release.sh
    
### Push the changelog

    git commit -m"meaningful message" files_that_changed 
    git push
    
### Create the Debian package

    ./scripts/build/build_debian.sh

## Build Avionics Debian

The `avionics` Debian is build separatly from the `freeflyer` Debian because it
does not rely on the same toolchain, follow a different release schedule and is
not yet open source.
    
### Update changelog

    cd $FF_AVIONICS
    dch -v <version.number> -c debian/changelog
  
### Push the new changelog

    git commit -m"meaningful message" debian/changelog
    git push
    
### Create the Debian package

    ./tools/build_debian.sh
    # answer y/n when the prompt ask about Harmony install

## Common steps: staging and snapshoting

### Stage the Debian on volar
    cd ..
    export ASTROBEE_DEBIAN_DIR=/home/p-free-flyer/free-flyer/FSW/ars_debs/dists/xenial/main
    scp astrobee-avionics_<version>_*.deb \
      <ndc_username>@volar:$ASTROBEE_DEBIAN_DIR/binary-armhf

### Update the Debian mirror on volar

   ssh volar
   cd /home/p-free-flyer/free-flyer/FSW/aptly/scripts
   ./update_astrobee_debians.sh 
   
Warnings [!] will be displayed for the existing packages, but the new generated 
debian should be listed with a [+].
  
### Sign the Debian

This step is only required if you do not already have imported the key
to sign the Debians.
This step requires to have access to the private key maintained by Brian Coltin.

Master key owner:
    gpg --export-secret-key
    
New user:
    gpg --import ~bcoltin/secret.key

Master key owner deletes secret.key

### Push Debian to the server

    ./publish_snapshot.sh <snapshot_date_and_tag>
    # convention for the argument: follow this example
    # 20180702_pre-dock-flight-release
    #
    # Input the Master Password when asked (twice).
    
## Build FSW dependencies Debians packages

When a FSW dependecy is changed, it is necessary to rebuild the Debian packages:

    scripts/setup/debians/build_install_debians.sh
    
Then, follow the common step above to stage the Debian packages on volar and 
create the snapshot on the server.


    