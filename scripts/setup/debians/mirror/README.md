# Debian Snapshoting and Mirroring
 
## General Functioning

The system is using [aptly](https://www.aptly.info/doc/overview/) to maintain a
database of required Debian. Aptly is configured on volar. To publish the latest
of the Debians on the local PPA, a snapshot is exported and synched to the
`astrobee.ndc.nasa.gov` server.

## Setup

In this directory, add the following files:

- `aptly`: Download the aptly software from https://www.aptly.info
- `astrobee_conf`: Directory containing the latest multistrap config files from
  `submodules/platform/rootfs`. This can be a symlink.
- `aptly.conf`: config file for aptly. On volar, we override the rootDir.

Update the Astrobee Debian files in:

    `$ASTROBEE_DEBIAN_DIR/binary-armhf`
    `$ASTROBEE_DEBIAN_DIR/binary-amd64`

By default, `$ASTROBEE_DEBIAN_DIR` is
`/home/p-free-flyer/free-flyer/FSW/ars_debs/dists/xenial/main`
for volar.

Also, you must add the Astrobee private key to GPG. When you figure out how to
do this, please update the instructions...

## Creating Initial Repository

1. Copy the latest `submodules/platform/rootfs/*.conf` files into scripts/astrobee_conf.
2. Copy the latest Debians we made to /home/p-free-flyer/free-flyer/FSW/ars_debs/dists/xenial/main/binary-armhf and binary-amd64.
3. Run `create_repo.sh`. This creates the initial 
4. Run `update_repo.sh`. This creates mirrors of the latest ubuntu repository based on the config files.
5. Run `publish\_snapshot.sh SNAPSHOT_NAME`, choosing an appropriate name (by convention, YYYYMMDD_description). This downloads packages from the mirrors and publishes a snapshot both to this directory and to astrobee.ndc.nasa.gov.

## Updating Files in packages.conf

Start from step 4.

## Updating Only Local Debians

Install new Debians to our local repository. Then run
```
update_astrobee_debians.sh 
```

(already called inside "update_repo.sh") and publish with step 5.

